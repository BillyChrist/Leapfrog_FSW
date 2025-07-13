# PowerShell install script for Windows
Write-Host "Leapfrog GroundStation Windows Setup"

Write-Host "\n1. Download and install the Protocol Buffers compiler (protoc) for Windows:"
Write-Host "   https://github.com/protocolbuffers/protobuf/releases"
Write-Host "   - Download the latest release for Windows (zip)"
Write-Host "   - Extract and add the 'bin' directory to your PATH"

Write-Host "\n2. (Optional) For C++ development, download or build the protobuf C++ library for your compiler."
Write-Host "   See: https://github.com/protocolbuffers/protobuf/blob/main/cmake/README.md"

Write-Host "\n3. Installing Python dependencies (protobuf, PyQt5, pyqtgraph, pyserial)..."
pip install --upgrade pip
pip install protobuf pyqt5 pyqtgraph pyserial

Write-Host "\nAll Python dependencies installed!"
Write-Host "\nIf you plan to build the C++ groundstation on Windows, ensure you have a compatible C++ toolchain and the protobuf library." 