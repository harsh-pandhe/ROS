# Contributing to ROS 2 Humble Workspace

Thank you for your interest in contributing! This document provides guidelines and instructions for contributing to this repository.

## Code of Conduct

- Be respectful and constructive in all interactions
- Welcome all contributors regardless of background
- Focus on the code, not the person
- Report issues or concerns to the maintainers

## How to Contribute

### 1. Fork and Clone

```bash
# Fork the repository on GitHub
# Clone your fork
git clone https://github.com/YOUR-USERNAME/ROS.git
cd ros2_humble_ws
```

### 2. Create a Branch

```bash
# Create a feature branch with a descriptive name
git checkout -b feature/add-new-node
# or for bug fixes:
git checkout -b fix/turtle-motion-issue
```

### 3. Make Changes

- Follow Python PEP 8 style guidelines
- Write clear, descriptive commit messages
- Test your changes thoroughly
- Update documentation if needed

### 4. Test Your Changes

```bash
# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Run relevant tests
colcon test

# Test your nodes manually with TurtleSim if applicable
```

### 5. Commit and Push

```bash
# Stage your changes
git add .

# Commit with a descriptive message
git commit -m "Add new turtle control node with improved motion"

# Push to your fork
git push origin feature/add-new-node
```

### 6. Create a Pull Request

- Go to GitHub and create a PR from your fork to the main repository
- Write a clear description of your changes
- Reference any related issues (e.g., "Fixes #123")
- Wait for review and feedback

## Contribution Guidelines

### Code Style

- Follow **PEP 8** for Python code
- Use meaningful variable and function names
- Add docstrings to functions and classes
- Keep lines under 100 characters where practical

### Commit Messages

Write clear commit messages following this format:

```
Brief description (50 chars or less)

More detailed explanation if needed. Wrap at 72 characters.
Explain what and why, not how.

Fixes #123
```

### Documentation

- Update README files when adding new features
- Document new nodes with docstrings
- Include usage examples for new packages
- Keep documentation up-to-date

### Testing

- Test new code before submitting
- Ensure existing tests still pass: `colcon test`
- Add tests for new functionality when applicable
- Test with ROS 2 Humble distribution

### Package Changes

If adding a new package:

1. Create a proper `package.xml` with metadata
2. Include a `setup.py` with entry points
3. Add a README.md specific to the package
4. Update the main workspace README.md

Example `package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_new_package</name>
  <version>0.0.1</version>
  <description>Brief description</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
</package>
```

## Types of Contributions

### Bug Reports
- Use GitHub Issues
- Describe the problem clearly
- Include steps to reproduce
- Mention your ROS 2 version and OS

### Feature Requests
- Use GitHub Issues with a clear title
- Explain the use case
- Provide examples or pseudocode if helpful

### Documentation Improvements
- Fix typos or unclear explanations
- Add examples or tutorials
- Improve package READMEs
- No need to open an issue first

### New Nodes or Packages
- Open an issue first to discuss
- Ensure it fits the workspace purpose
- Follow the package structure guidelines
- Include comprehensive documentation

## Development Workflow

1. **Setup**: Clone, build, and test the workspace
2. **Create**: Make changes in your feature branch
3. **Test**: Run `colcon build` and `colcon test`
4. **Document**: Update relevant README files
5. **Commit**: Push changes to your fork
6. **Review**: Submit PR and respond to feedback
7. **Merge**: Once approved, changes are merged

## Useful Commands

```bash
# Build a specific package
colcon build --packages-select my_package_name

# Build with verbose output
colcon build --event-handlers console_direct+

# Run tests
colcon test

# List packages
ros2 pkg list

# Check topics
ros2 topic list

# Inspect a node
ros2 node list
```

## Questions or Issues?

- Open a GitHub Issue for bugs or feature requests
- Check existing Issues before creating a new one
- Be descriptive and provide reproducible examples
- Include error messages and logs

## Recognition

Contributors who make significant improvements will be acknowledged in the project.

Thank you for contributing to this ROS 2 learning workspace!
