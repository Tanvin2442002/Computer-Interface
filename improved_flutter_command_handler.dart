// // Updated handleCommand function with proper async/await
// Future<void> handleCommand(String command) async {
//   String botResponse;
//   bool shouldSendToESP32 = false;
//   String esp32Command = '';

//   // Add user message to chat first
//   setState(() {
//     chatMessages.add({'sender': 'You', 'message': command});
//   });

//   switch (command) {
//     case 'Deliver':
//       botResponse = 'Starting delivery mode! Stopping advertising and sending command...';
      
//       // Stop advertising first, then send command
//       try {
//         await stopAdvertising();
//         shouldSendToESP32 = true;
//         esp32Command = 'deliver';
//         botResponse = 'Delivery started! Advertising stopped, robot moving...';
//       } catch (e) {
//         botResponse = 'Failed to stop advertising: $e';
//       }
//       break;

//     case 'Meet me':
//       botResponse = 'Starting meeting mode! Beginning advertising and sending command...';
      
//       // Start advertising first, then send command
//       try {
//         await startAdvertising();
//         shouldSendToESP32 = true;
//         esp32Command = 'meet';
//         botResponse = 'Meeting started! Advertising active, robot coming to you...';
//       } catch (e) {
//         botResponse = 'Failed to start advertising: $e';
//       }
//       break;

//     case 'Test Robot':
//       botResponse = 'Testing robot connection...';
//       shouldSendToESP32 = true;
//       esp32Command = 'test';
//       break;

//     case 'Stop Robot':
//       botResponse = 'Stopping all robot movement.';
//       shouldSendToESP32 = true;
//       esp32Command = 'stop';
//       break;

//     case 'Rotate Robot':
//       botResponse = 'Robot will rotate 360 degrees.';
//       shouldSendToESP32 = true;
//       esp32Command = 'rotate';
//       break;

//     case 'Move Forward':
//       botResponse = 'Robot moving forward...';
//       shouldSendToESP32 = true;
//       esp32Command = 'forward';
//       break;

//     case 'Goodbye':
//       botResponse = 'Goodbye! Stopping robot and advertising...';
      
//       // Stop both robot and advertising
//       try {
//         await stopAdvertising();
//         shouldSendToESP32 = true;
//         esp32Command = 'stop';
//         botResponse = 'Goodbye! Robot stopped and advertising disabled.';
//       } catch (e) {
//         botResponse = 'Goodbye! Robot stopped but advertising error: $e';
//         shouldSendToESP32 = true;
//         esp32Command = 'stop';
//       }
//       break;

//     default:
//       botResponse = 'I don\'t understand that command.';
//   }

//   // Add bot response to chat
//   setState(() {
//     chatMessages.add({'sender': 'Bot', 'message': botResponse});
//   });

//   // Send command to ESP32 if needed
//   if (shouldSendToESP32) {
//     if (!isConnectedToESP32) {
//       // Try to connect first
//       await testESP32Connection();
//       if (isConnectedToESP32) {
//         await sendCommandToESP32(esp32Command);
//       }
//     } else {
//       await sendCommandToESP32(esp32Command);
//     }
//   }
// }

// // Also update the button press to handle async
// // In your widget build method, update the buttons like this:
// ElevatedButton(
//   onPressed: () async {
//     await handleCommand(command);  // Add await here
//   },
//   style: ElevatedButton.styleFrom(
//     backgroundColor: Colors.deepOrange,
//     padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
//   ),
//   child: Text(
//     command,
//     style: TextStyle(color: Colors.white, fontSize: 12),
//   ),
// ),

// // Optional: Add loading state to prevent multiple commands
// bool isProcessingCommand = false;

// Future<void> handleCommand(String command) async {
//   // Prevent multiple commands at once
//   if (isProcessingCommand) {
//     setState(() {
//       chatMessages.add({'sender': 'Bot', 'message': 'Please wait, processing previous command...'});
//     });
//     return;
//   }

//   setState(() {
//     isProcessingCommand = true;
//   });

//   try {
//     // ... rest of your handleCommand code ...
//   } finally {
//     setState(() {
//       isProcessingCommand = false;
//     });
//   }
// }

// // Update buttons to show loading state
// ElevatedButton(
//   onPressed: isProcessingCommand ? null : () async {
//     await handleCommand(command);
//   },
//   style: ElevatedButton.styleFrom(
//     backgroundColor: isProcessingCommand ? Colors.grey : Colors.deepOrange,
//     padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
//   ),
//   child: Text(
//     isProcessingCommand ? 'Processing...' : command,
//     style: TextStyle(color: Colors.white, fontSize: 12),
//   ),
// ),
