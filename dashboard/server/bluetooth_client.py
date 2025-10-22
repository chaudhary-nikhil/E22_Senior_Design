#!/usr/bin/env python3
"""
Bluetooth Low Energy client for GoldenForm device communication
"""

import asyncio
import json
import time
from bleak import BleakClient, BleakScanner
from .config import DEVICE_NAME, SERVICE_UUID, CHARACTERISTIC_UUID, BLE_SCAN_TIMEOUT, LOG_FORMAT

class BluetoothIMUReceiver:
    """Handles Bluetooth communication with GoldenForm device"""
    
    def __init__(self, data_callback=None):
        self.client = None
        self.connected = False
        self.data_buffer = ""
        self.data_callback = data_callback
        self.session_data_buffer = ""  # Buffer for large session data
        self.expecting_session_data = False
        self.session_data_size = 0
        self.received_session_size = 0
        
    def notification_handler(self, sender, data: bytearray):
        """Handle incoming BLE notifications"""
        try:
            # Convert bytes to string
            data_str = data.decode('utf-8')
            
            if self.expecting_session_data:
                # Handle large session data transmission
                self.session_data_buffer += data_str
                self.received_session_size += len(data_str)
                
                # Calculate progress
                if self.session_data_size > 0:
                    progress = (self.received_session_size / self.session_data_size) * 100
                    print(f"📊 Session data progress: {progress:.1f}% ({self.received_session_size}/{self.session_data_size} bytes)")
                
                # Check if we've received all session data
                if self.session_data_size > 0 and self.received_session_size >= self.session_data_size:
                    print("✅ Complete session data received!")
                    self._process_complete_session_data()
                    self._reset_session_buffer()
                return
            
            # Handle regular real-time data
            self.data_buffer += data_str
            
            # Try to find complete JSON objects
            while True:
                try:
                    # Find the end of a JSON object
                    json_end = self.data_buffer.find('}')
                    if json_end == -1:
                        break  # No complete JSON yet
                    
                    # Extract JSON string
                    json_str = self.data_buffer[:json_end + 1]
                    self.data_buffer = self.data_buffer[json_end + 1:]
                    
                    # Parse JSON
                    data_obj = json.loads(json_str)
                    
                    # Call the data callback if provided
                    if self.data_callback:
                        self.data_callback(data_obj)
                    
                except json.JSONDecodeError:
                    # Invalid JSON, skip this part
                    break
                except Exception as e:
                    print(f"Error processing data: {e}")
                    break
                    
        except Exception as e:
            print(f"Error in notification handler: {e}")
    
    def _process_complete_session_data(self):
        """Process complete session data once received"""
        try:
            # Parse the complete session JSON
            session_data = json.loads(self.session_data_buffer)
            
            # Call the data callback with the complete session
            if self.data_callback:
                self.data_callback(session_data)
                
        except json.JSONDecodeError as e:
            print(f"❌ Failed to parse session data: {e}")
        except Exception as e:
            print(f"❌ Error processing session data: {e}")
    
    def _reset_session_buffer(self):
        """Reset session data buffer"""
        self.session_data_buffer = ""
        self.expecting_session_data = False
        self.session_data_size = 0
        self.received_session_size = 0
    
    def start_session_data_reception(self, expected_size=0):
        """Start receiving large session data"""
        print(f"📡 Starting session data reception (expected: {expected_size} bytes)")
        self.expecting_session_data = True
        self.session_data_size = expected_size
        self.session_data_buffer = ""
        self.received_session_size = 0
    
    async def scan_and_connect(self):
        """Scan for and connect to the GoldenForm device"""
        print("🔍 Scanning for Bluetooth devices...")
        devices = await BleakScanner.discover(timeout=BLE_SCAN_TIMEOUT)
        
        target_address = None
        for device in devices:
            print(f"Found device: {device.name or 'Unknown'} ({device.address})")
            if device.name and DEVICE_NAME.lower() in device.name.lower():
                print(f"✅ Found target device: {device.name} at {device.address}")
                target_address = device.address
                break
        
        if not target_address:
            print(f"❌ Device '{DEVICE_NAME}' not found")
            return False
        
        print(f"🔗 Connecting to {target_address}...")
        
        try:
            async with BleakClient(target_address) as client:
                self.client = client
                self.connected = True
                
                print("✅ Connected successfully!")
                
                # Find our characteristic
                services = await client.get_services()
                char = None
                for service in services:
                    for characteristic in service.characteristics:
                        if str(characteristic.uuid).lower() == CHARACTERISTIC_UUID.lower():
                            char = characteristic
                            break
                    if char:
                        break
                
                if not char:
                    print(f"❌ Characteristic {CHARACTERISTIC_UUID} not found")
                    return False
                
                print(f"📡 Found characteristic: {char.uuid}")
                
                # Enable notifications
                await client.start_notify(char, self.notification_handler)
                print("📊 Notifications enabled - receiving IMU data...")
                
                # Keep connection alive
                while self.connected:
                    await asyncio.sleep(1)
                
        except Exception as e:
            print(f"❌ Bluetooth connection error: {e}")
            return False
        finally:
            self.connected = False
    
    def disconnect(self):
        """Disconnect from Bluetooth device"""
        self.connected = False

async def bluetooth_logger(data_callback):
    """Bluetooth logger task"""
    receiver = BluetoothIMUReceiver(data_callback)
    
    while True:
        try:
            await receiver.scan_and_connect()
        except Exception as e:
            print(f"Bluetooth receiver error: {e}")
        
        print("🔄 Reconnecting in 5 seconds...")
        await asyncio.sleep(5)
