// Copyright (c) 2025 Damien Boisvert (AlphaGameDeveloper)
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

package dev.alphagame;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class LogManager {
    private static final DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss.SSS");
    private static final boolean DEBUG_ENABLED = true;
    

    /**
     * Formats a log message with the current timestamp, log level, caller information, and message.
     * 
     * @param level The log level (e.g. INFO, DEBUG, WARN, ERROR, FATAL)
     * @param message The log message
     * @return The formatted log message
     */
    private static String formatLogMessage(String level, String message) {
        // Get timestamp
        String timestamp = LocalDateTime.now().format(formatter);
        
        // Get caller information
        StackTraceElement caller = Thread.currentThread().getStackTrace()[3]; // [0] is getStackTrace, [1] is formatLogMessage, [2] is the log method, [3] is the caller
        String callerInfo = caller.getFileName() + ":" + caller.getLineNumber();
        
        // Format with fixed-width columns for better alignment
        return String.format("%-23s | %-7s | %-30s | %s", 
                             timestamp, 
                             level, 
                             callerInfo, 
                             message);
    }

    /**
     * Logs a debug message.
     * 
     * @param message The message to log
     * @see #formatLogMessage(String, String)
     */
    public static void debug(String message) {
        if (!DEBUG_ENABLED) return; // Skip debug messages if not enabled
        System.out.println(formatLogMessage("DEBUG", message));
    }
    
    /**
     * Logs an informational message.
     * 
     * @param message The message to log
     * @see #formatLogMessage(String, String)
     */
    public static void info(String message) {
        System.out.println(formatLogMessage("INFO", message));
    }
    
    /**
     * Logs a warning message.
     * 
     * @param message The message to log
     * @see #formatLogMessage(String, String)
     */
    public static void warning(String message) {
        System.out.println(formatLogMessage("WARN", message));
    }
    
    /**
     * Logs an error message.
     * 
     * @param message The message to log
     * @see #formatLogMessage(String, String)
     */
    public static void error(String message) {
        System.err.println(formatLogMessage("ERROR", message));
    }
    
    /**
     * Logs a fatal error message.String
     * 
     * @param message The message to log
     * @see #formatLogMessage(String, String)
     */
    public static void fatal(String message) {
        System.err.println(formatLogMessage("FATAL", message));
    }
}