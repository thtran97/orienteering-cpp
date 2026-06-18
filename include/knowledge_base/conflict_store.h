#pragma once

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace oplib::knowledge_base {

enum class KBVal : uint8_t { F, T, U };

struct KBVar {
    int  id        = 0;
    int  weight    = 0;
    bool watchable = true;
    std::unordered_set<int> watching_set;  // conflict IDs this var is watching
    std::unordered_set<int> active_set;    // conflict IDs blocking this var
    std::unordered_set<int> expl;          // conflicts this var explains
    std::string to_string() const;
};

struct KBConflict {
    int              id;
    std::vector<int> scope;   // boolean variable IDs in the conflict
    int              w1 = -1; // first  watched literal
    int              w2 = -1; // second watched literal
    bool             active = false;
    std::string to_string() const;
};

/**
 * Conflict store with a 2-watched-literal scheme.
 *
 * Variables represent the multi-vehicle assignment [client c → vehicle v].
 * Each boolean variable bx = v * nb_clients + c  (1-based c, 0-based v).
 *
 * A conflict { bx1, bx2, ..., bxk } means the corresponding joint assignment
 * is time-window infeasible: at most k-1 of them can simultaneously hold.
 *
 * Direct port of kb_factory::boolvar::wl_cflt::ConflictStore from kb_ls_cpp,
 * adapted to the orienteering-cpp namespace and build system.
 */
class ConflictStore {
public:
    ConflictStore(int nb_clients, int nb_vehicles = 1);
    ~ConflictStore() = default;

    // ---------- assignment management ----------
    bool assign(int c, int v);   // mark client c as assigned to vehicle v
    void unassign(int c);        // release all vehicle assignments of c

    // ---------- conflict learning ----------
    void add_conflict(std::vector<int>& client_scope); // scope = client IDs (1-based)

    // ---------- feasibility queries ----------
    bool check_assign(int c, int v) const;  // can client c still go in vehicle v?

    // ---------- diagnostics ----------
    int  size()  const { return nb_conflicts_; }
    void reset();
    void print_assignment() const;

private:
    int nb_clients_;
    int nb_vehicles_;
    int nb_boolvars_;   // nb_clients_ * nb_vehicles_
    int nb_conflicts_ = 0;

    std::vector<KBVar>      vars_;        // indexed [0..nb_boolvars_]  (1-based bx)
    std::vector<KBVal>      assignment_;  // indexed [0..nb_boolvars_]  (1-based bx)
    std::vector<KBConflict> conflicts_;   // indexed [0..nb_conflicts_-1]

    // temporary pointer reused in hot paths
    KBConflict* ptr_conflict_ = nullptr;
    KBVar*      ptr_var_      = nullptr;

    // ---------- encoding ----------
    int  encode(int c, int v)  const { return v * nb_clients_ + c; }

    // ---------- private helpers ----------
    bool assign_bool(int bx, bool v);
    void unassign_bool(int bx);
    void add_conflict_bool(std::vector<int>& scope);
    void activate_var(int bx);
    void deactivate_var(int bx);
    void remove_watchable_var(int bx, int cid);
    void add_watchable_var(int bx, int cid);
    int  get_new_watchable_var(const KBConflict& c) const;
    bool can_be_true(int bx) const;
};

} // namespace oplib::knowledge_base
