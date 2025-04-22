// Copyright (c) 2025 Damien Boisvert (AlphaGameDeveloper)
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

package dev.alphagame.rampage;

import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import dev.alphagame.trailblazer.LogManager;

public class Rampage {
    private static final ScheduledExecutorService memoryMonitorExecutor = Executors.newSingleThreadScheduledExecutor();

    public static void startMemoryMonitor(Double threshold) {
        LogManager.info("Starting Rampage, the memory monitor from heck.");
        LogManager.info("Memory monitor started. Monitoring memory usage... (t: " + threshold + ")");
        memoryMonitorExecutor.scheduleAtFixedRate(() -> {
            MemoryMXBean memoryBean = ManagementFactory.getMemoryMXBean();
            MemoryUsage heapUsage = memoryBean.getHeapMemoryUsage();
            long usedMemory = heapUsage.getUsed();
            long maxMemory = heapUsage.getMax();

            if (usedMemory > maxMemory * threshold) { // Example threshold: 80% of max memory
                LogManager.warning("High memory usage detected!");
                LogManager.warning("Used Memory: " + usedMemory + " bytes (" + (usedMemory / (1024 * 1024)) + " MB)");
                LogManager.warning("Max Memory: " + maxMemory + " bytes (" + (maxMemory / (1024 * 1024)) + " MB)");
                LogManager.warning("Heap Memory Usage: " + heapUsage);
                LogManager.debug("Heap Memory Usage (Used): " + heapUsage.getUsed() + " bytes");

                LogManager.warning("Using " + (usedMemory / (double) maxMemory) * 100 + "% of max memory");
            }
            LogManager.debug("m_used_bytes: %s, m_max_bytes: %s, m_used_percent: %s, m_threshold: %s", usedMemory, maxMemory, (usedMemory / (double) maxMemory) * 100, threshold);
        }, 0, 10, TimeUnit.SECONDS); // Check every 10 seconds
    }

    public static void stopMemoryMonitor() {
        memoryMonitorExecutor.shutdown();
    }
}

