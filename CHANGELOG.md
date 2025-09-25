# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Development status notice in README files
- Comprehensive .gitignore file for Arduino/PlatformIO projects
- Updated CLAUDE.md with complete project structure

### Changed
- Corrected license information from LGPL v2.1 to MIT in README files

### Fixed
- License consistency between README files and LICENSE file

## [0.0.1] - 2023-XX-XX

### Added
- Initial release with complete Adaptive Fuzzy PID implementation
- Seven membership function types (Triangle, Trapezoid, Gaussian, BellShaped, SShaped, ZShaped, Singleton)
- Four inference modes (MamdaniMaxMin, MamdaniMaxProduct, TSK, SAM)
- Real-time PID parameter adaptation based on fuzzy logic
- Configurable sample time (1ms - 30s)
- Anti-windup protection for integral term
- Input/output range configuration
- Comprehensive parameter validation
- Non-blocking operation using Arduino's millis()
- Basic and advanced usage examples
- English and Thai documentation
- Arduino library structure compliance
- MIT License
- Code of Conduct

### Technical Details
- Fuzzy logic engine with rule-based system
- Seven linguistic variables for error classification
- Smooth parameter transitions
- Efficient fuzzy inference algorithms
- Memory-optimized implementation for Arduino platforms

[Unreleased]: https://github.com/dhanabhon/AdaptiveFuzzyPID/compare/v0.0.1...HEAD
[0.0.1]: https://github.com/dhanabhon/AdaptiveFuzzyPID/releases/tag/v0.0.1