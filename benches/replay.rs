use sprouts::replay::{self, ms, Fixture, MoveTiming, PHASES};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

const RECORD_NODES: [usize; 3] = [4, 5, 6];
const RECORD_BOARDS: [usize; 2] = [1000, 500];

fn fixtures_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures")
}

fn games_dir() -> PathBuf {
    fixtures_dir().join("games")
}

fn game_paths() -> Result<Vec<PathBuf>, String> {
    let dir = games_dir();
    let mut paths: Vec<PathBuf> = std::fs::read_dir(&dir)
        .map_err(|e| format!("{}: {}", dir.display(), e))?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|p| p.extension().is_some_and(|e| e == "json"))
        .collect();
    paths.sort();
    Ok(paths)
}

fn load_budgets() -> Result<BTreeMap<String, f64>, String> {
    let path = fixtures_dir().join("budgets.json");
    let json = std::fs::read_to_string(&path).map_err(|e| format!("{}: {}", path.display(), e))?;
    let budgets: BTreeMap<String, f64> = serde_json::from_str(&json).map_err(|e| e.to_string())?;
    for phase in PHASES {
        if !budgets.contains_key(phase) {
            return Err(format!("{}: no budget for {}", path.display(), phase));
        }
    }
    Ok(budgets)
}

fn print_table(name: &str, fixture: &Fixture, timings: &[MoveTiming]) -> [f64; 6] {
    println!(
        "\n{name}: {} nodes, board {}, {} moves",
        fixture.initial_nodes,
        fixture.board_size,
        fixture.moves.len()
    );
    print!("{:>4} {:>6} {:>5}", "move", "player", "over");
    for phase in PHASES {
        print!(" {phase:>14}");
    }
    println!(" {:>10}", "total");

    for t in timings {
        let player = match (t.terminal, t.player) {
            (true, _) => "end",
            (false, replay::Player::Human) => "human",
            (false, replay::Player::AI) => "ai",
        };
        print!("{:>4} {:>6} {:>5}", t.index, player, t.game_over);
        for d in t.phases.as_array() {
            print!(" {:>14.2}", ms(d));
        }
        println!(" {:>10.2}", ms(t.phases.total()));
    }
    let max = replay::max_per_phase(timings);
    print!("{:>4} {:>6} {:>5}", "max", "", "");
    for v in max {
        print!(" {v:>14.2}");
    }
    println!(" {:>10.2}", max.iter().sum::<f64>());
    max
}

fn run_bench() -> Result<bool, String> {
    let budgets = load_budgets()?;
    let paths = game_paths()?;
    if paths.is_empty() {
        return Err(format!("no fixtures under {}", games_dir().display()));
    }

    let mut failures = Vec::new();
    for path in &paths {
        let name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("fixture");
        match replay::load(path) {
            Ok(fixture) => {
                let run = replay::replay(&fixture);
                let max = print_table(name, &fixture, &run.timings);
                failures.extend(replay::breaches(name, &max, &budgets));
                if let Some(e) = run.failure {
                    println!("{name}: {e}");
                    failures.push(format!("{name}: {e}"));
                }
            }
            Err(e) => {
                println!("\n{name}: {e}");
                failures.push(format!("{name}: {e}"));
            }
        }
    }

    let (forms, hits, misses, (lives, nodes)) = replay::nimber_stats();
    let rate = 100.0 * hits as f64 / (hits + misses).max(1) as f64;
    println!(
        "\nnimbers: {forms} forms stored, {hits} hits, {misses} misses ({rate:.1}% hit rate), \
         largest region solved {nodes} nodes with {lives} lives"
    );

    let (positions, took) = sprouts::search_rate();
    println!(
        "\nsearch rate at 20 nodes: {positions} positions in {took:.0} ms, {:.0} positions/s",
        positions as f64 / took * 1000.0
    );

    println!("\nbudgets (ms):");
    for phase in PHASES {
        println!("  {phase:>14} {:>10.2}", budgets[phase]);
    }
    if failures.is_empty() {
        println!("\nall phases within budget");
    } else {
        println!("\nfailures:");
        for f in &failures {
            println!("  {f}");
        }
    }
    Ok(failures.is_empty())
}

fn run_record() -> Result<(), String> {
    let dir = games_dir();
    std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {}", dir.display(), e))?;
    for nodes in RECORD_NODES {
        for board in RECORD_BOARDS {
            let fixture = replay::record(nodes, board);
            let path = dir.join(format!("selfplay-{nodes}n-{board}.json"));
            if fixture.moves.is_empty() {
                println!(
                    "{}: skipped, the AI has no opening move here",
                    path.display()
                );
                continue;
            }
            replay::write(&fixture, &path)?;
            println!("{}: {} moves", path.display(), fixture.moves.len());
        }
    }
    Ok(())
}

/// Prints the positions the search charges to reach each depth, summed over
/// the AI's choices in every fixture.
fn run_nodes_per_depth(max_depth: usize) -> Result<(), String> {
    let paths = game_paths()?;
    print!("{:>28}", "fixture");
    for depth in 1..=max_depth {
        print!(" {:>9}", format!("depth {depth}"));
    }
    println!();
    for path in &paths {
        let name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("fixture");
        let fixture = replay::load(path)?;
        print!("{name:>28}");
        for total in replay::nodes_per_depth(&fixture, max_depth) {
            print!(" {total:>9}");
        }
        println!();
    }
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let record = args.iter().any(|a| a == "--record");
    let depths = args
        .iter()
        .position(|a| a == "--nodes-per-depth")
        .map(|at| args.get(at + 1).and_then(|d| d.parse().ok()).unwrap_or(4));
    let outcome = if record {
        run_record().map(|()| true)
    } else if let Some(max_depth) = depths {
        run_nodes_per_depth(max_depth).map(|()| true)
    } else {
        run_bench()
    };
    match outcome {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}
