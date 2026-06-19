#include <algorithm>
#include <cassert>
#include <iostream>
#include <sstream>

#include "knowledge_base/conflict_store.h"

namespace oplib::knowledge_base {

// ---------------------------------------------------------------------------
// KBVar / KBConflict helpers
// ---------------------------------------------------------------------------

std::string KBVar::to_string() const {
    std::ostringstream ss;
    ss << "Var" << id << "(w=" << weight << ",watch={";
    for (int c : watching_set) ss << c << ",";
    ss << "},active={";
    for (int c : active_set) ss << c << ",";
    ss << "})";
    return ss.str();
}

std::string KBConflict::to_string() const {
    std::ostringstream ss;
    ss << "Conflict" << id << "(scope={";
    for (int s : scope) ss << s << ",";
    ss << "},w1=" << w1 << ",w2=" << w2 << ",active=" << active << ")";
    return ss.str();
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

ConflictStore::ConflictStore(int nb_clients, int nb_vehicles)
    : nb_clients_(nb_clients), nb_vehicles_(nb_vehicles),
      nb_boolvars_(nb_clients * nb_vehicles)
{
    // Index 0 unused; valid bx in [1..nb_boolvars_]
    vars_.resize(nb_boolvars_ + 1);
    assignment_.resize(nb_boolvars_ + 1, KBVal::U);
    for (int bx = 1; bx <= nb_boolvars_; ++bx)
        vars_[bx].id = bx;
}

// ---------------------------------------------------------------------------
// reset
// ---------------------------------------------------------------------------

void ConflictStore::reset() {
    for (int bx = 1; bx <= nb_boolvars_; ++bx) {
        assignment_[bx] = KBVal::U;
        vars_[bx].watchable  = true;
        vars_[bx].watching_set.clear();
        vars_[bx].active_set.clear();
        vars_[bx].expl.clear();
    }
    for (auto& cft : conflicts_) {
        cft.active = false;
        cft.w1     = -1;
        cft.w2     = -1;
    }
}

// ---------------------------------------------------------------------------
// Public: assign / unassign
// ---------------------------------------------------------------------------

bool ConflictStore::assign(int c, int v) {
    int bx = encode(c, v);
    return assign_bool(bx, true);
}

void ConflictStore::unassign(int c) {
    for (int v = 0; v < nb_vehicles_; ++v) {
        int bx = encode(c, v);
        if (assignment_[bx] == KBVal::T)
            unassign_bool(bx);
    }
}

// ---------------------------------------------------------------------------
// Public: add_conflict
// ---------------------------------------------------------------------------

void ConflictStore::add_conflict(std::vector<int>& client_scope) {
    // For each vehicle, add a conflict clause in boolean-variable space.
    // The conflict "clients {c1, c2, ..., ck} cannot all be in vehicle v"
    // is encoded as: NOT(bx(c1,v)) OR NOT(bx(c2,v)) OR ... OR NOT(bx(ck,v)).
    for (int v = 0; v < nb_vehicles_; ++v) {
        std::vector<int> bool_scope;
        bool_scope.reserve(client_scope.size());
        for (int c : client_scope)
            bool_scope.push_back(encode(c, v));
        add_conflict_bool(bool_scope);
    }
}

// ---------------------------------------------------------------------------
// Public: check_assign
// ---------------------------------------------------------------------------

bool ConflictStore::check_assign(int c, int v) const {
    int bx = encode(c, v);
    return can_be_true(bx);
}

// ---------------------------------------------------------------------------
// Public: print_assignment
// ---------------------------------------------------------------------------

void ConflictStore::print_assignment() const {
    for (int c = 1; c <= nb_clients_; ++c) {
        for (int v = 0; v < nb_vehicles_; ++v) {
            int bx = encode(c, v);
            char ch = (assignment_[bx] == KBVal::T) ? 'T'
                    : (assignment_[bx] == KBVal::F) ? 'F' : 'U';
            std::cout << "c" << c << "v" << v << "=" << ch << " ";
        }
    }
    std::cout << '\n';
}

// ---------------------------------------------------------------------------
// Private: assign_bool
// ---------------------------------------------------------------------------

bool ConflictStore::assign_bool(int bx, bool val) {
    if (val) {
        if (assignment_[bx] == KBVal::T) return true;   // already assigned
        assignment_[bx] = KBVal::T;
        activate_var(bx);
    } else {
        if (assignment_[bx] == KBVal::F) return true;
        assignment_[bx] = KBVal::F;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Private: unassign_bool
// ---------------------------------------------------------------------------

void ConflictStore::unassign_bool(int bx) {
    if (assignment_[bx] == KBVal::T)
        deactivate_var(bx);
    assignment_[bx] = KBVal::U;
}

// ---------------------------------------------------------------------------
// Private: add_conflict_bool
// ---------------------------------------------------------------------------

void ConflictStore::add_conflict_bool(std::vector<int>& scope) {
    KBConflict cft;
    cft.id    = nb_conflicts_;
    cft.scope = scope;

    // Find two watchable (not-yet-true) variables to watch.
    int watched_count = 0;
    for (int bx : scope) {
        if (vars_[bx].watchable) {
            if (watched_count == 0)
                cft.w1 = bx;
            else
                cft.w2 = bx;
            vars_[bx].watching_set.insert(cft.id);
            ++watched_count;
            if (watched_count == 2) break;
        }
    }

    if (watched_count < 2) {
        // Less than 2 watchable vars → conflict is already active.
        cft.active = true;
        for (int bx : scope) {
            if (bx != cft.w1)
                vars_[bx].active_set.insert(cft.id);
        }
        if (cft.w1 != -1)
            vars_[cft.w1].expl.insert(cft.id);
    }

    conflicts_.push_back(std::move(cft));
    ++nb_conflicts_;
}

// ---------------------------------------------------------------------------
// Private: activate_var  (bx has just been set to TRUE)
// ---------------------------------------------------------------------------

void ConflictStore::activate_var(int bx) {
    vars_[bx].watchable = false;
    // For each conflict this var was watching, find a new watcher.
    std::vector<int> to_process(vars_[bx].watching_set.begin(),
                                 vars_[bx].watching_set.end());
    for (int cid : to_process)
        remove_watchable_var(bx, cid);
}

// ---------------------------------------------------------------------------
// Private: deactivate_var  (bx has just been unassigned)
// ---------------------------------------------------------------------------

void ConflictStore::deactivate_var(int bx) {
    vars_[bx].watchable = true;
    // For each conflict bx was in active_set: try to become a new watcher.
    std::vector<int> to_process(vars_[bx].active_set.begin(),
                                 vars_[bx].active_set.end());
    for (int cid : to_process)
        add_watchable_var(bx, cid);
}

// ---------------------------------------------------------------------------
// Private: remove_watchable_var  (bx leaves conflict cid's watch)
// ---------------------------------------------------------------------------

void ConflictStore::remove_watchable_var(int bx, int cid) {
    KBConflict& cft = conflicts_[cid];
    int new_w = get_new_watchable_var(cft);

    if (new_w != -1) {
        // Replace bx with new_w as watcher.
        if (cft.w1 == bx) cft.w1 = new_w;
        else               cft.w2 = new_w;
        vars_[bx].watching_set.erase(cid);
        vars_[new_w].watching_set.insert(cid);
    } else {
        // No replacement → conflict becomes active.
        vars_[bx].watching_set.erase(cid);
        cft.active = true;
        // The remaining watcher (w_keep) becomes the explanation literal.
        int w_keep = (cft.w1 == bx) ? cft.w2 : cft.w1;
        for (int s : cft.scope) {
            if (s != w_keep)
                vars_[s].active_set.insert(cid);
        }
        if (w_keep != -1)
            vars_[w_keep].expl.insert(cid);
    }
}

// ---------------------------------------------------------------------------
// Private: add_watchable_var  (bx becomes watchable again, tries to join cid)
// ---------------------------------------------------------------------------

void ConflictStore::add_watchable_var(int bx, int cid) {
    KBConflict& cft = conflicts_[cid];
    if (!cft.active) return;
    if (!vars_[bx].watchable) return;

    // bx can become the second watcher → deactivate conflict.
    cft.active = false;
    vars_[bx].active_set.erase(cid);
    vars_[bx].watching_set.insert(cid);

    // Find the explanation literal: the watcher whose expl set contains cid.
    // (The other watcher in {w1,w2} is the one that triggered activation.)
    int w_keep = -1;
    if (cft.w1 != -1 && vars_[cft.w1].expl.count(cid))
        w_keep = cft.w1;
    else if (cft.w2 != -1 && vars_[cft.w2].expl.count(cid))
        w_keep = cft.w2;

    // Replace the non-explanation watcher slot with bx.
    if (w_keep == cft.w1)
        cft.w2 = bx;
    else
        cft.w1 = bx;

    // Remove explanation from w_keep and clear all active_sets.
    if (w_keep != -1) {
        vars_[w_keep].expl.erase(cid);
        for (int s : cft.scope) {
            if (s != w_keep)
                vars_[s].active_set.erase(cid);
        }
    }
}

// ---------------------------------------------------------------------------
// Private: get_new_watchable_var
// ---------------------------------------------------------------------------

int ConflictStore::get_new_watchable_var(const KBConflict& cft) const {
    for (int bx : cft.scope) {
        if (bx == cft.w1 || bx == cft.w2) continue;
        if (vars_[bx].watchable) return bx;
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Private: can_be_true
// ---------------------------------------------------------------------------

bool ConflictStore::can_be_true(int bx) const {
    if (assignment_[bx] == KBVal::T) return true;
    if (assignment_[bx] == KBVal::F) return false;
    // Unassigned: feasible iff no active conflict pins it to false.
    if (vars_[bx].expl.empty()) return true;
    // Track which clauses are actually firing (VSIDS-style activity).
    for (int cid : vars_[bx].expl)
        ++conflicts_[cid].activity;
    return false;
}

// ---------------------------------------------------------------------------
// Public: compact
// ---------------------------------------------------------------------------

int ConflictStore::compact(int min_activity) {
    // Collect bool-var-encoded scopes of clauses meeting the activity threshold.
    std::vector<std::vector<int>> kept;
    int removed = 0;
    for (int i = 0; i < nb_conflicts_; ++i) {
        if (conflicts_[i].activity >= min_activity)
            kept.push_back(conflicts_[i].scope);
        else
            ++removed;
    }

    if (removed == 0) return 0;

    // Reset all var/conflict state (assignments cleared too — caller must kb_sync).
    for (int bx = 1; bx <= nb_boolvars_; ++bx) {
        assignment_[bx] = KBVal::U;
        vars_[bx].watchable = true;
        vars_[bx].watching_set.clear();
        vars_[bx].active_set.clear();
        vars_[bx].expl.clear();
    }
    conflicts_.clear();
    nb_conflicts_ = 0;

    // Re-add surviving clauses (activity resets to 0 on the fresh KBConflict).
    for (auto& scope : kept)
        add_conflict_bool(scope);

    return removed;
}

} // namespace oplib::knowledge_base
