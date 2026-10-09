mod analysis;
mod app;
mod editor;
mod library;
mod renderer;
mod scene_file;
mod scenes;
mod sim;

use winit::{
    application::ApplicationHandler,
    event::WindowEvent,
    event_loop::{ActiveEventLoop, EventLoop, EventLoopBuilder, EventLoopProxy},
    window::{Window, WindowId},
};

/// Sent when the async `App::new` finishes. Native builds block on it inside
/// `resumed`, but the browser can't block, so there it arrives as an event.
#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
pub enum UserEvent {
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
        } else if let (Some(app), Some(window)) = (&mut self.app, self.window) {
            // Back from the background (Android): a fresh surface for the
            // window; the world and everything else carried on.
            app.render_state.recreate_surface(window);
            app.resize(window.inner_size());
            window.request_redraw();
        }
    }

    fn suspended(&mut self, _event_loop: &ActiveEventLoop) {
        // Android destroys the window's surface while the app is away.
        if let Some(app) = &mut self.app {
            app.render_state.drop_surface();
        }
    }

    fn user_event(&mut self, _event_loop: &ActiveEventLoop, event: UserEvent) {
        match event {
            UserEvent::Ready(app) => {
                let app = self.app.insert(*app);
                if let Some(window) = self.window {
                    // The canvas got its real size (a `Resized` event) while
                    // `App::new` was still awaiting the GPU, when there was no
                    // app to receive it; catch up now.
                    app.resize(window.inner_size());
                    window.request_redraw();
                }
                // Fade out index.html's loading screen; the first frame
                // lands underneath it while it fades.
                #[cfg(target_arch = "wasm32")]
                if let Some(el) = web_sys::window()
                    .and_then(|w| w.document())
                    .and_then(|d| d.get_element_by_id("loading"))
                {
                    let _ = el.set_attribute("class", "done");
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
            // Suspended: no surface to draw on, and no redraw loop until resume.
            WindowEvent::RedrawRequested if app.render_state.surface.is_none() => {}
            WindowEvent::RedrawRequested => {
                app.update();
                match app.render() {
                    Ok(_) => {}
                    Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                        let size = app.window_size;
                        app.render_state.resize(size);
                    }
                    Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                    #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
                    Err(e) => eprintln!("render error: {e:?}"),
                    #[cfg(any(target_arch = "wasm32", target_os = "android"))]
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

/// Desktop and web entry point (main.rs).
pub fn main() {
    #[cfg(target_arch = "wasm32")]
    {
        std::panic::set_hook(Box::new(console_error_panic_hook::hook));
        let _ = console_log::init_with_level(log::Level::Warn);
    }
    run(EventLoop::<UserEvent>::with_user_event());
}

/// Android entry point: the NativeActivity loads this library and calls it.
#[cfg(target_os = "android")]
#[unsafe(no_mangle)]
fn android_main(android: winit::platform::android::activity::AndroidApp) {
    use winit::platform::android::EventLoopBuilderExtAndroid;
    android_logger::init_once(android_logger::Config::default().with_max_level(log::LevelFilter::Warn).with_tag("qonstraint"));
    std::panic::set_hook(Box::new(|info| log::error!("{info}")));
    if let Some(dir) = android.internal_data_path() {
        library::set_data_dir(dir);
    }
    let mut builder = EventLoop::<UserEvent>::with_user_event();
    builder.with_android_app(android);
    run(builder);
}

fn run(mut builder: EventLoopBuilder<UserEvent>) {
    let event_loop = builder.build().unwrap();
    // In the browser `request_redraw` already runs a frame per animation
    // frame; polling there only floods the main thread with wake-ups, which
    // starves Firefox's animation frames.
    #[cfg(not(target_arch = "wasm32"))]
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
