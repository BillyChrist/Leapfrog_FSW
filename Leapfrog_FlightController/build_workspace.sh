# This executable will clean and rebuild the workspace ensuring correct build order.
#!/bin/bash

# Execute in workspace root with: ./build_workspace.sh

set -e # Exits immediately if any command fails


# =============================
# 1. Clean old build artifacts
# =============================
echo "🔄 Cleaning old build artifacts..."
rm -rf build/ install/ log/

# =====================================
# 2. Build core libraries individually
# =====================================
echo "🔧 Building 'communication' package..."
colcon build --symlink-install --packages-select communication

# =============================
# 3. Source environment
# =============================
echo "✅ Sourcing workspace..."
source install/setup.bash

# =====================================
# 4. Build core application (flightcontrol)
# =====================================
echo "🔧 Building flightcontrol packages..."
colcon build --symlink-install --packages-up-to flightcontrol --event-handlers console_direct+

# =====================================
# 5. Build STM32 bridge
# =====================================
echo "🔧 Building 'stm32_bridge' packages..."
colcon build --symlink-install --packages-up-to stm32_bridge --event-handlers console_direct+


# =============================
# 6. Build remaining packages
# =============================
echo "🔧 Building remaining packages..."
colcon build --symlink-install --packages-skip communication flightcontrol stm32_bridge

# =====================================
# 7. Build top-level workspace
# =====================================
echo "🔧 Building FlightController workspace..."
colcon build --symlink-install --packages-select leapfrog_flightcontroller --event-handlers console_direct+

# =============================
# 8. Optional: Run Tests
# =============================
# echo "🧪 Running tests..."
# colcon test
# colcon test-result --verbose

# =============================
# 9. Done
# =============================
echo "✅ Build complete."