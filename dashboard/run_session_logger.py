#!/usr/bin/env python3
"""
GoldenForm Session Logger - Main Entry Point
Run this from the dashboard directory to start the session logger
"""

import sys
import os

# Add server directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'server'))

# Import and run the main application
from server.session_logger import main

if __name__ == "__main__":
    main()
