#!/bin/sh
# Android build: `./android.sh build` makes the APK, `./android.sh run`
# installs it on the phone plugged in over USB (debugging on) and starts it.
# Uses the user-local rustup in ~/.cargo (the system Rust has no Android
# target) and the Android Studio SDK and NDK.
set -e
export PATH="$HOME/.cargo/bin:$PATH"
export ANDROID_HOME="${ANDROID_HOME:-$HOME/Android/Sdk}"
export ANDROID_SDK_ROOT="$ANDROID_HOME"
export ANDROID_NDK_ROOT="${ANDROID_NDK_ROOT:-$(ls -d "$ANDROID_HOME"/ndk/* | sort -V | tail -1)}"
# Release (the physics needs it) signed with the standard Android debug key,
# which Android Studio creates; fine for installing on your own devices.
export CARGO_APK_RELEASE_KEYSTORE="${CARGO_APK_RELEASE_KEYSTORE:-$HOME/.android/debug.keystore}"
export CARGO_APK_RELEASE_KEYSTORE_PASSWORD="${CARGO_APK_RELEASE_KEYSTORE_PASSWORD:-android}"
cmd="${1:-build}"
[ $# -gt 0 ] && shift
exec cargo apk "$cmd" --lib --release "$@"
