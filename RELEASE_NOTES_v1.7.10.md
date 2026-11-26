# Release Notes v1.7.10

IRK Capture v1.7.10 adds support for ESP-IDF 6. There are no new entities, options or Status values.

## What's new for you

- **Builds on ESP-IDF 6.** Choosing ESP-IDF 6 in your YAML (for example `version: 6.1.1`) no longer fails with `mbedtls/aes.h: No such file or directory`. IRK Check works the same way on ESP-IDF 5 and 6. Thanks to David Coulson for the fix.

### Behind the scenes

- Every supported board (ESP32, C3, C6 and S3) is now built with both ESP-IDF 5.5.6 and 6.1.1 on every pull request, so a break on either version is caught before release.

## Upgrading

- **No YAML changes are needed.** The shipped packages and ESPHome's default still use ESP-IDF 5.x. ESP-IDF 6 is optional.
- **ESPHome 2026.9.0 or newer** is still required.
- **Rebuild as usual.** Devices that track `main` pick up this version on their next build.
- **Scripts that call the wizard API:** no changes.
