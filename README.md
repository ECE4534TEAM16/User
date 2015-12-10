# User Rover
This code runs a "user" rover for ECE 4534 Fall 2015, Team 16.

This code will set up a rover to wait for a start signal, upon which it will follow a black line until it reaches an intersection. At that point, the rover will wait until it receives a character (l, r, f, or E) via UART and make the appropriate turn. If it does not receive the end signal, it will continue running.
