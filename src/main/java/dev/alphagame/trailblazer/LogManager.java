// Copyright (c) 2025 Damien Boisvert (AlphaGameDeveloper)
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

package dev.alphagame.trailblazer;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

/*
 * LogManager.java
 * 
 * This class provides a simple logging utility for people too lazy to use Log4J.
 * It formats log messages with a timestamp, log level, and caller information.
 */
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
     * Formats a log message with arguments.
     * 
     * @param level The log level (e.g. INFO, DEBUG, WARN, ERROR, FATAL)
     * @param message The log message with format specifiers
     * @param args The arguments to replace format specifiers
     * @return The formatted log message
     */
    private static String formatLogMessage(String level, String message, Object... args) {
        // Get timestamp
        String timestamp = LocalDateTime.now().format(formatter);

        // Get caller information
        StackTraceElement caller = Thread.currentThread().getStackTrace()[3];
        String callerInfo = caller.getFileName() + ":" + caller.getLineNumber();

        // Format the message with arguments
        String formattedMessage = String.format(message, args);

        // Format with fixed-width columns for better alignment
        return String.format("%-23s | %-7s | %-30s | %s",
                             timestamp,
                             level,
                             callerInfo,
                             formattedMessage);
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
     * Logs a debug message with arguments.
     * 
     * @param message The message to log with format specifiers
     * @param args The arguments to replace format specifiers
     */
    public static void debug(String message, Object... args) {
        if (!DEBUG_ENABLED) return; // Skip debug messages if not enabled
        System.out.println(formatLogMessage("DEBUG", message, args));
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
     * Logs an informational message with arguments.
     * 
     * @param message The message to log with format specifiers
     * @param args The arguments to replace format specifiers
     */
    public static void info(String message, Object... args) {
        System.out.println(formatLogMessage("INFO", message, args));
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
     * Logs a warning message with arguments.
     * 
     * @param message The message to log with format specifiers
     * @param args The arguments to replace format specifiers
     */
    public static void warning(String message, Object... args) {
        System.out.println(formatLogMessage("WARN", message, args));
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
     * Logs an error message with arguments.
     * 
     * @param message The message to log with format specifiers
     * @param args The arguments to replace format specifiers
     */
    public static void error(String message, Object... args) {
        System.err.println(formatLogMessage("ERROR", message, args));
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

    /**
     * Logs a fatal error message with arguments.
     * 
     * @param message The message to log with format specifiers
     * @param args The arguments to replace format specifiers
     */
    public static void fatal(String message, Object... args) {
        System.err.println(formatLogMessage("FATAL", message, args));
    }
}