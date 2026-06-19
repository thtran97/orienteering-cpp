#include <set>

#include "bench_utils.h"

namespace bench {

const std::vector<std::string> ALL_VARIANTS = {
    "op", "top", "toptw", "tdop", "tdoptw", "ttdp", "mctopmtw", "singlesat"};

namespace {

std::vector<std::string> split_csv(const std::string& s)
{
    std::vector<std::string> out;
    std::istringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ','))
        if (!token.empty()) out.push_back(token);
    return out;
}

std::string detect_variant(const fs::path& filepath,
                           const std::vector<std::string>& known)
{
    fs::path p = filepath.parent_path();
    while (!p.empty() && p != p.parent_path()) {
        std::string name = p.filename().string();
        std::string lower = name;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
        for (const auto& v : known)
            if (lower == v) return v;
        p = p.parent_path();
    }
    return "";
}

std::vector<InstanceSpec> find_tdop_instances(const fs::path& tdop_root)
{
    std::vector<InstanceSpec> result;
    if (!fs::is_directory(tdop_root)) return result;

    std::string speed_matrix = (tdop_root / "speedmatrix.txt").string();
    if (!fs::exists(speed_matrix)) return result;

    for (const auto& ds : fs::directory_iterator(tdop_root)) {
        if (!ds.is_directory()) continue;
        std::string arc_cat;
        for (const auto& e : fs::directory_iterator(ds.path())) {
            std::string n = e.path().filename().string();
            if (n.rfind("arc_cat", 0) == 0) { arc_cat = e.path().string(); break; }
        }
        if (arc_cat.empty()) continue;

        fs::path op_dir = ds.path() / "OP_instances";
        if (!fs::is_directory(op_dir)) continue;

        for (const auto& inst : fs::directory_iterator(op_dir)) {
            if (!inst.is_regular_file()) continue;
            result.push_back({inst.path().string(), "tdop", speed_matrix, arc_cat});
        }
    }
    std::sort(result.begin(), result.end(),
              [](const InstanceSpec& a, const InstanceSpec& b) {
                  return a.filepath < b.filepath; });
    return result;
}

std::vector<InstanceSpec> find_tdoptw_instances(const fs::path& tdoptw_root)
{
    std::vector<InstanceSpec> result;
    if (!fs::is_directory(tdoptw_root)) return result;

    std::map<std::string, std::string> trans_map;
    for (const auto& e : fs::directory_iterator(tdoptw_root)) {
        if (!e.is_regular_file()) continue;
        std::string n = e.path().filename().string();
        if (n.rfind("titt", 0) == 0) {
            std::string size_str = n.substr(4);
            auto dot = size_str.find('.');
            if (dot != std::string::npos) size_str = size_str.substr(0, dot);
            trans_map[size_str] = e.path().string();
        }
    }

    for (const auto& e : fs::directory_iterator(tdoptw_root)) {
        if (!e.is_regular_file()) continue;
        std::string n = e.path().filename().string();
        if (n.rfind("titt", 0) == 0) continue;
        if (n.rfind("tt",   0) == 0) continue;
        if (n.rfind("TD_",  0) == 0) continue;
        if (n.rfind("OPTW_",0) == 0) continue;

        auto dot = n.find('.');
        if (dot == std::string::npos) continue;
        std::string size_str = n.substr(0, dot);

        auto it = trans_map.find(size_str);
        if (it == trans_map.end()) continue;

        result.push_back({e.path().string(), "tdoptw", it->second, ""});
    }
    std::sort(result.begin(), result.end(),
              [](const InstanceSpec& a, const InstanceSpec& b) {
                  return a.filepath < b.filepath; });
    return result;
}

} // anonymous namespace

// ============================================================================
// discover_instances
// ============================================================================

std::vector<InstanceSpec> discover_instances(
    const std::string& root_path,
    const std::vector<std::string>& variants)
{
    std::vector<InstanceSpec> result;
    fs::path root(root_path);

    if (!fs::exists(root)) {
        std::cerr << "[ERROR] Path does not exist: " << root_path << '\n';
        return result;
    }

    bool want_tdop   = variants.empty() ||
        std::find(variants.begin(), variants.end(), "tdop")   != variants.end();
    bool want_tdoptw = variants.empty() ||
        std::find(variants.begin(), variants.end(), "tdoptw") != variants.end();

    auto add_file = [&](const fs::path& fp) {
        std::string var = detect_variant(fp, ALL_VARIANTS);
        if (var.empty()) return;
        if (var == "tdop" || var == "tdoptw") return;
        if (!variants.empty() &&
            std::find(variants.begin(), variants.end(), var) == variants.end())
            return;
        result.push_back({fp.string(), var});
    };

    if (fs::is_regular_file(root)) {
        std::string var = detect_variant(root, ALL_VARIANTS);
        if (!var.empty() && var != "tdop" && var != "tdoptw") {
            if (variants.empty() ||
                std::find(variants.begin(), variants.end(), var) != variants.end())
                result.push_back({root.string(), var});
        }
    } else if (fs::is_directory(root)) {
        for (const auto& entry : fs::recursive_directory_iterator(root)) {
            if (entry.is_regular_file()) add_file(entry.path());
        }
        if (want_tdop) {
            fs::path tdop_dir = root / "tdop";
            if (!fs::exists(tdop_dir)) tdop_dir = root;
            auto tdop = find_tdop_instances(tdop_dir);
            result.insert(result.end(), tdop.begin(), tdop.end());
        }
        if (want_tdoptw) {
            fs::path tdoptw_dir = root / "tdoptw";
            if (!fs::exists(tdoptw_dir)) tdoptw_dir = root;
            auto tdoptw = find_tdoptw_instances(tdoptw_dir);
            result.insert(result.end(), tdoptw.begin(), tdoptw.end());
        }
        std::sort(result.begin(), result.end(),
                  [](const InstanceSpec& a, const InstanceSpec& b) {
                      return a.filepath < b.filepath; });
    }
    return result;
}

// ============================================================================
// quick_filter
// ============================================================================

std::vector<InstanceSpec> quick_filter(
    std::vector<InstanceSpec> instances, int n)
{
    std::map<std::string, int> variant_count;
    std::vector<InstanceSpec> trimmed;
    for (const auto& spec : instances) {
        if (variant_count[spec.variant] < n) {
            trimmed.push_back(spec);
            ++variant_count[spec.variant];
        }
    }
    return trimmed;
}

// ============================================================================
// parse_instance
// ============================================================================

std::unique_ptr<Problem> parse_instance(const InstanceSpec& spec)
{
    const std::string& path = spec.filepath;
    try {
        if (spec.variant == "op")        { OPParser p;        return p.read(path); }
        if (spec.variant == "top")       { TOPParser p;       return p.read(path); }
        if (spec.variant == "toptw")     { TOPTWParser p;     return p.read(path); }
        if (spec.variant == "ttdp")      { TTDPParser p;      return p.read(path); }
        if (spec.variant == "mctopmtw")  { MCTOPMTWParser p;  return p.read(path); }
        if (spec.variant == "singlesat") { SingleSatParser p; return p.read(path); }
        if (spec.variant == "tdop") {
            if (spec.aux1.empty() || spec.aux2.empty())
                throw std::runtime_error("TDOP requires speed_matrix and arc_cat");
            return TDOPParser::read(path, spec.aux1, spec.aux2);
        }
        if (spec.variant == "tdoptw") {
            if (spec.aux1.empty())
                throw std::runtime_error("TDOPTW requires transition_matrix");
            return TDOPTWParser::read(path, spec.aux1);
        }
    } catch (const std::exception& e) {
        std::cerr << "[WARN] Parse error (" << spec.variant << "): "
                  << spec.filepath << " -- " << e.what() << '\n';
    }
    return nullptr;
}

// ============================================================================
// apply_overrides
// ============================================================================

void apply_overrides(const Options& opts, Problem& problem)
{
    if (opts.vehicles > 0)
        problem.set_num_vehicles(opts.vehicles);
}

// ============================================================================
// CSV I/O
// ============================================================================

void write_header(std::ofstream& out)
{
    out << "Solver,Instance,Variant,Nodes,Vehicles,HasTW,IsTimeDep,"
           "Budget,Run,Reward,TravelTime,CustomersVisited,CPU_ms,Status\n";
}

void write_row(std::ofstream& out, const RunResult& r)
{
    out << r.solver               << ','
        << r.instance_name        << ','
        << r.variant              << ','
        << r.num_nodes            << ','
        << r.num_vehicles         << ','
        << (r.has_time_windows    ? "yes" : "no") << ','
        << (r.is_time_dependent   ? "yes" : "no") << ','
        << std::fixed << std::setprecision(4)
        << r.budget               << ','
        << r.run_id               << ','
        << r.reward               << ','
        << r.travel_time          << ','
        << r.customers_visited    << ','
        << r.cpu_ms               << ','
        << r.status               << '\n';
}

// ============================================================================
// open_csv_files
// ============================================================================

std::map<std::string, std::ofstream> open_csv_files(
    const std::string& output_root,
    const std::string& solver_name,
    const std::vector<std::string>& variants,
    bool overwrite)
{
    std::map<std::string, std::ofstream> files;
    for (const auto& variant : variants) {
        fs::path dir = fs::path(output_root) / variant;
        if (!fs::exists(dir)) {
            try { fs::create_directories(dir); }
            catch (const std::exception& e) {
                std::cerr << "[ERROR] Cannot create " << dir << ": " << e.what() << '\n';
                continue;
            }
        }

        fs::path csv_path = dir / ("benchmark_" + solver_name + ".csv");
        auto open_mode = overwrite || !fs::exists(csv_path)
                         ? (std::ios::out | std::ios::trunc)
                         : (std::ios::out | std::ios::app);

        auto& csv = files[variant];
        csv.open(csv_path.string(), open_mode);
        if (!csv.is_open()) {
            std::cerr << "[ERROR] Cannot open " << csv_path.string() << '\n';
            continue;
        }
        if (open_mode == (std::ios::out | std::ios::trunc))
            write_header(csv);

        std::cout << "  " << csv_path.string() << '\n';
    }
    return files;
}

// ============================================================================
// close_csv_files
// ============================================================================

void close_csv_files(std::map<std::string, std::ofstream>& files)
{
    for (auto& [variant, csv] : files)
        if (csv.is_open()) csv.close();
}

// ============================================================================
// CLI parsing
// ============================================================================

namespace {

void print_help(const std::string& solver_name)
{
    std::cout <<
R"(Orienteering per-solver benchmark

Usage: bench_<solver> [options]

Options:
  --variant   -v  <name[,...]>   Variants to include. (default: all)
  --instance  -i  <path>         File or directory of instances.  (default: data/)
  --timeout   -t  <sec>          CPU time limit per run.         (default: 60)
  --iterations    <n>            Max iterations per solver.      (default: 1000)
  --seed          <n>            Random seed.                    (default: 42)
  --runs          <n>            Runs per solver/instance pair.  (default: 1)
  --output    -o  <folder>       Root output folder.             (default: results/)
  --overwrite                    Overwrite existing CSV files.
  --quick     -q  [<n>]          First n instances per variant only.
  --pulse-labels  <n>            Label budget for pulse solver.  (default: 0)
  --vehicles      <n>            Override number of vehicles.    (default: from file)
  --verbose                      Pass verbose flag to solver.
  --help      -h                 Show this help.

Solver: )" << solver_name << R"(

Variants: op  top  toptw  tdop  tdoptw  ttdp  mctopmtw  singlesat

Output: results/<variant>/benchmark_<solver>.csv
)";
}

} // anonymous namespace

Options parse_cli(int argc, char** argv, const std::string& solver_name)
{
    Options opts;
    bool variants_set = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];

        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + a);
            return argv[++i];
        };

        if (a == "--help" || a == "-h") { print_help(solver_name); std::exit(0); }
        else if (a == "--verbose")       opts.verbose   = true;
        else if (a == "--overwrite")      opts.overwrite = true;
        else if (a == "--quick" || a == "-q") {
            if (i + 1 < argc && std::isdigit(static_cast<unsigned char>(argv[i+1][0])))
                opts.quick = std::stoi(argv[++i]);
            else
                opts.quick = 3;
        }
        else if (a == "--variant" || a == "-v") {
            auto names = split_csv(next());
            if (!variants_set) { opts.variants.clear(); variants_set = true; }
            for (auto& n : names) opts.variants.push_back(n);
        }
        else if (a == "--instance"    || a == "-i") opts.instance_path = next();
        else if (a == "--timeout"     || a == "-t") opts.timeout       = std::stod(next());
        else if (a == "--output"      || a == "-o") opts.output_root   = next();
        else if (a == "--iterations")                opts.iterations   = std::stoi(next());
        else if (a == "--seed")                      opts.seed         = std::stoi(next());
        else if (a == "--runs")                      opts.runs         = std::stoi(next());
        else if (a == "--pulse-labels")              opts.pulse_labels = std::stoi(next());
        else if (a == "--vehicles")                  opts.vehicles     = std::stoi(next());
        else if (a == "--alpha")                     opts.alpha        = std::stoi(next());
        else if (a == "--rcl-size")                  opts.rcl_size     = std::stoi(next());
        else {
            std::cerr << "[WARN] Unknown argument: " << a << '\n';
        }
    }

    if (opts.variants.empty())
        opts.variants = ALL_VARIANTS;

    const std::set<std::string> valid_var(ALL_VARIANTS.begin(), ALL_VARIANTS.end());
    for (const auto& v : opts.variants) {
        if (!valid_var.count(v)) {
            std::cerr << "[ERROR] Unknown variant: '" << v << "'.\n";
            std::exit(1);
        }
    }

    return opts;
}

} // namespace bench
