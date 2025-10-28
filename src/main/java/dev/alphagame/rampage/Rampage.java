// Copyright (c) 2025 Damien Boisvert (AlphaGameDeveloper)
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

package dev.alphagame.rampage;

import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;
import java.lang.management.GarbageCollectorMXBean;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import dev.alphagame.trailblazer.LogManager;

public class Rampage {
    private static final ScheduledExecutorService memoryMonitorExecutor = Executors.newSingleThreadScheduledExecutor();
    private static long lastGcCount = 0;

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

            // Check garbage collection stats
            long totalGcCount = 0;
            @SuppressWarnings("unused")
            long totalGcTime = 0;
            for (GarbageCollectorMXBean gcBean : ManagementFactory.getGarbageCollectorMXBeans()) {
                totalGcCount += gcBean.getCollectionCount();
                totalGcTime += gcBean.getCollectionTime();
            }

            long gcCountDiff = totalGcCount - lastGcCount;
            long bytesFreed = 0;
            if (gcCountDiff > 0) {
                bytesFreed = (lastGcCount > 0) ? (maxMemory - usedMemory) : 0; // Estimate bytes freed

            }

            lastGcCount = totalGcCount;
            LogManager.debug("m_used_bytes: %s, m_max_bytes: %s, m_used_percent: %s, m_threshold: %s, m_gc_feed_estimate: %s (%s MB), m_gc_count: %s", usedMemory, maxMemory, (usedMemory / (double) maxMemory) * 100, threshold, bytesFreed, (bytesFreed / (1024*1024)), gcCountDiff);

        }, 0, 10, TimeUnit.SECONDS); // Check every 10 seconds
    }

    public static void stopMemoryMonitor() {
        memoryMonitorExecutor.shutdown();
    }
}

