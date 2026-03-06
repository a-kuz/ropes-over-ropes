use std::time::Instant;
use uzls_cross::level::generator;

fn main() {
    let levels: Vec<u32> = (1..=200).collect();
    let warmup = 3;
    let iterations = 50;

    println!("=== Level Generator Benchmark ===");
    println!("Levels: 1..200, warmup: {warmup}, iterations: {iterations}\n");

    for _ in 0..warmup {
        for &id in &levels {
            std::hint::black_box(generator::generate(id));
        }
    }

    let mut per_level: Vec<(u32, f64)> = Vec::new();

    for &id in &levels {
        let start = Instant::now();
        for _ in 0..iterations {
            std::hint::black_box(generator::generate(id));
        }
        let elapsed = start.elapsed();
        let avg_us = elapsed.as_secs_f64() * 1_000_000.0 / iterations as f64;
        per_level.push((id, avg_us));
    }

    per_level.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

    println!("Top 20 slowest levels:");
    println!("{:>6} {:>12} {:>10}", "Level", "Avg (µs)", "Avg (ms)");
    println!("{:-<30}", "");
    for &(id, us) in per_level.iter().take(20) {
        println!("{:>6} {:>12.1} {:>10.3}", id, us, us / 1000.0);
    }

    let total_us: f64 = per_level.iter().map(|(_, us)| us).sum();
    let avg_us = total_us / per_level.len() as f64;
    let min = per_level.last().unwrap();
    let max = per_level.first().unwrap();

    println!("\n--- Summary (all 200 levels) ---");
    println!("  Average: {:.1} µs ({:.3} ms)", avg_us, avg_us / 1000.0);
    println!("  Min:     {:.1} µs (level {})", min.1, min.0);
    println!("  Max:     {:.1} µs (level {})", max.1, max.0);
    println!(
        "  Total (1 pass, 200 levels): {:.1} µs ({:.2} ms)",
        total_us,
        total_us / 1000.0
    );

    println!("\n--- By difficulty tier ---");
    let tiers: &[(std::ops::RangeInclusive<u32>, &str)] = &[
        (1..=2, "1-2 (3 ropes, 3 drags)"),
        (3..=5, "3-5 (4 ropes, 4 drags)"),
        (6..=9, "6-9 (5 ropes, 6 drags)"),
        (10..=15, "10-15 (6 ropes, 8 drags)"),
        (16..=25, "16-25 (7 ropes, 10 drags)"),
        (26..=50, "26-50 (8 ropes, 12+ drags)"),
        (51..=200, "51-200 (8-10 ropes, 16+ drags)"),
    ];
    for (range, label) in tiers {
        let tier_times: Vec<f64> = per_level
            .iter()
            .filter(|(id, _)| range.contains(id))
            .map(|(_, us)| *us)
            .collect();
        if tier_times.is_empty() {
            continue;
        }
        let tier_avg = tier_times.iter().sum::<f64>() / tier_times.len() as f64;
        let tier_max = tier_times.iter().cloned().fold(0.0f64, f64::max);
        println!(
            "  {:<30} avg={:.1}µs  max={:.1}µs",
            label, tier_avg, tier_max
        );
    }

    println!("\n--- Per-layout average ---");
    let layout_names = [
        "Grid4x5",
        "Circle12",
        "Hexagon",
        "Diamond",
        "Cross",
        "TwoRings",
        "Triangle",
        "Star",
        "Grid5x6",
        "Circle16",
        "HexagonLarge",
        "DiamondWide",
        "CrossLarge",
        "ThreeRings",
        "TriangleLarge",
        "StarLarge",
        "Honeycomb",
        "Spiral",
        "DoubleGrid",
        "Scattered",
    ];
    for (layout_idx, name) in layout_names.iter().enumerate() {
        let layout_times: Vec<f64> = per_level
            .iter()
            .filter(|(id, _)| *id as usize % 20 == layout_idx)
            .map(|(_, us)| *us)
            .collect();
        if layout_times.is_empty() {
            continue;
        }
        let layout_avg = layout_times.iter().sum::<f64>() / layout_times.len() as f64;
        println!(
            "  {:<16} avg={:.1}µs  (n={})",
            name,
            layout_avg,
            layout_times.len()
        );
    }
}
