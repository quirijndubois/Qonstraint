mod analysis;
mod app;
mod editor;
mod renderer;
mod scene_file;
mod scenes;
mod sim;

use winit::{
    application::ApplicationHandler,
    event::WindowEvent,
    event_loop::{ActiveEventLoop, EventLoop, EventLoopProxy},
    window::{Window, WindowId},
};

/// Sent when the async `App::new` finishes. Native builds block on it inside
/// `resumed`, but the browser can't block, so there it arrives as an event.
#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
enum UserEvent {
    Ready(Box<app::App>),
}

struct PhysicsApp {
    app: Option<app::App>,
    window: Option<&'static Window>,
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    proxy: EventLoopProxy<UserEvent>,
}

impl ApplicationHandler<UserEvent> for PhysicsApp {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_none() {
            let attrs = Window::default_attributes().with_title("Physics Sim");
            #[cfg(not(target_arch = "wasm32"))]
            let attrs = attrs.with_inner_size(winit::dpi::PhysicalSize::new(1280u32, 720u32));
            // In the browser the canvas fills the page via CSS (index.html).
            #[cfg(target_arch = "wasm32")]
            let attrs = {
                use winit::platform::web::WindowAttributesExtWebSys;
                attrs.with_append(true).with_prevent_default(true)
            };
            let window = event_loop.create_window(attrs).unwrap();
            // SAFETY: we keep the window alive for the program's lifetime
            let window: &'static Window = Box::leak(Box::new(window));
            self.window = Some(window);

            #[cfg(not(target_arch = "wasm32"))]
            {
                let app = pollster::block_on(app::App::new(window));
                self.app = Some(app);
            }
            #[cfg(target_arch = "wasm32")]
            {
                let proxy = self.proxy.clone();
                wasm_bindgen_futures::spawn_local(async move {
                    let app = app::App::new(window).await;
                    let _ = proxy.send_event(UserEvent::Ready(Box::new(app)));
                });
            }
        }
    }

    fn user_event(&mut self, _event_loop: &ActiveEventLoop, event: UserEvent) {
        match event {
            UserEvent::Ready(app) => {
                self.app = Some(*app);
                if let Some(window) = self.window {
                    window.request_redraw();
                }
            }
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        let app = match &mut self.app {
            Some(a) => a,
            None => return,
        };

        match &event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::RedrawRequested => {
                app.update();
                match app.render() {
                    Ok(_) => {}
                    Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                        let size = app.window_size;
                        app.render_state.resize(size);
                    }
                    Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                    #[cfg(not(target_arch = "wasm32"))]
                    Err(e) => eprintln!("render error: {e:?}"),
                    #[cfg(target_arch = "wasm32")]
                    Err(e) => log::error!("render error: {e:?}"),
                }
                self.window.unwrap().request_redraw();
            }
            _ => {
                app.handle_event(&event);
            }
        }
    }
}

fn main() {
    #[cfg(target_arch = "wasm32")]
    {
        std::panic::set_hook(Box::new(console_error_panic_hook::hook));
        let _ = console_log::init_with_level(log::Level::Warn);
    }

    let event_loop = EventLoop::<UserEvent>::with_user_event().build().unwrap();
    event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);
    let proxy = event_loop.create_proxy();
    #[cfg_attr(target_arch = "wasm32", allow(unused_mut))]
    let mut physics_app =PhysicsApp { app: None, window: None, proxy };

    #[cfg(not(target_arch = "wasm32"))]
    event_loop.run_app(&mut physics_app).unwrap();
    // The browser owns the loop: returns immediately and keeps running.
    #[cfg(target_arch = "wasm32")]
    {
        use winit::platform::web::EventLoopExtWebSys;
        event_loop.spawn_app(physics_app);
    }
}
