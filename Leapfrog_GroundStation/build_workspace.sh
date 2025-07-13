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
# 2. Build ground station package
# =====================================
echo "🔧 Building 'leapfrog_groundstation' package..."
colcon build --symlink-install --packages-select leapfrog_groundstation --event-handlers console_direct+

# =============================
# 3. Source environment
# =============================
echo "✅ Sourcing workspace..."
source install/setup.bash

# =============================
# 4. Optional: Run Tests
# =============================
# echo "🧪 Running tests..."
# colcon test
# colcon test-result --verbose

# =============================
# 5. Done
# =============================
echo "✅ Build complete."
echo "To run the ground station:"
echo "1. Source the workspace: source install/setup.bash"
echo "2. Run the node: ros2 run leapfrog_groundstation ground_station" 