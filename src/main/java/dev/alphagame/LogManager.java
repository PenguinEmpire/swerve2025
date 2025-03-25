// Copyright (c) 2025 Damien Boisvert (AlphaGameDeveloper)
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

package dev.alphagame;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class LogManager {
    private static final DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss.SSS");
    
    private static String formatLogMessage(String level, String message) {
        // Get timestamp
        String timestamp = LocalDateTime.now().format(formatter);
        
        // Get caller information
        StackTraceElement caller = Thread.currentThread().getStackTrace()[3]; // [0] is getStackTrace, [1] is formatLogMessage, [2] is the log method, [3] is the caller
        String callerInfo = caller.getFileName() + ":" + caller.getLineNumber();
        
        return String.format("[%s] [%s] [%s] %s", timestamp, level, callerInfo, message);
    }

    public static void info(String message) {
        System.out.println(formatLogMessage("INFO", message));
    }
    
    public static void debug(String message) {
        System.out.println(formatLogMessage("DEBUG", message));
    }
    
    public static void warning(String message) {
        System.out.println(formatLogMessage("WARN", message));
    }
    
    public static void error(String message) {
        System.err.println(formatLogMessage("ERROR", message));
    }
    
    public static void fatal(String message) {
        System.err.println(formatLogMessage("FATAL", message));
    }
}