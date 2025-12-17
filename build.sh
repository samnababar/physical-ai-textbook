#!/bin/bash
set -e

echo "Building Docusaurus textbook from textbook directory..."
cd textbook
npm run build
echo "Build completed successfully!"
