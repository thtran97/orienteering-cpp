#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>

#include "knowledge_base/sel_manager.h"

namespace oplib::knowledge_base {

// --- Var Implementation ---

Var::Var(int id, int profit) : id(id), profit(profit), value(-1) {}

bool Var::has_value() const {
    return value != -1;
}

bool Var::set(bool v, std::vector<int>& propagated_assignment) {
    value = v ? 1 : 0;
    for (size_t i = 0; i < involving_constraints.size(); i++) {
        Constraint* ctr = involving_constraints[i];
        if (ctr->is_inconsistent()) return false;
        if (ctr->is_satisfied()) continue;

        ctr->revise_imp_score(this, involving_constraints_pos[i], true);
        bool ok = ctr->propagate(propagated_assignment);
        if (!ok) return false;
    }
    return true;
}

void Var::reset() {
    value = -1;
}

double Var::calculate_imp_score() {
    return static_cast<double>(profit);
}

// --- Constraint Implementation ---

Constraint::Constraint(int id, std::vector<int>& coeff, std::vector<Var*> scope, bool less_than, int rhs)
    : id(id), less_than_ct(less_than), rhs(rhs), coeff(coeff), scope(scope), constraint_weight(-1.0) {}

bool Constraint::is_inconsistent() {
    if (less_than_ct) {
        return accumulated_lhs > rhs;
    } else {
        return expected_lhs + accumulated_lhs < rhs;
    }
}

bool Constraint::is_satisfied() {
    if (less_than_ct) {
        return expected_lhs <= slack;
    } else {
        return accumulated_lhs >= rhs;
    }
}

void Constraint::calculate_imp_score(double weighted_coeff) {
    weighted_lhs = 0;
    expected_lhs = 0;
    accumulated_lhs = 0;
    for (size_t i = 0; i < scope.size(); i++) {
        Var* x = scope[i];
        if (x->has_value()) {
            accumulated_lhs += x->value * coeff[i];
        } else {
            expected_lhs += coeff[i];
            weighted_lhs += x->profit * coeff[i];
        }
    }
    slack = static_cast<double>(rhs) - accumulated_lhs;
    if (constraint_weight < 0.0) {
        constraint_weight = weighted_coeff;
    }
    if (less_than_ct) {
        calculate_imp_score_lt();
    } else {
        calculate_imp_score_gt();
    }
}

void Constraint::revise_imp_score(Var* x, int p, bool is_set) {
    if (is_set) {
        expected_lhs -= coeff[p];
        accumulated_lhs += x->value * coeff[p];
        weighted_lhs -= x->profit * coeff[p];
    } else {
        expected_lhs += coeff[p];
        accumulated_lhs -= x->value * coeff[p];
        weighted_lhs += x->profit * coeff[p];
    }
    slack = static_cast<double>(rhs) - accumulated_lhs;
    if (less_than_ct) {
        revise_imp_score_lt(x, p);
    } else {
        revise_imp_score_gt(x, p);
    }
}

bool Constraint::propagate(std::vector<int>& propagated_assignment) {
    if (is_inconsistent()) return false;
    if (less_than_ct) {
        return propagate_lt(propagated_assignment);
    } else {
        return propagate_gt(propagated_assignment);
    }
}

void Constraint::calculate_imp_score_lt() {
    if (expected_lhs <= slack || slack == 0) {
        imp_score = 0;
    } else {
        imp_score = constraint_weight * weighted_lhs / slack;
    }
}

void Constraint::revise_imp_score_lt(Var* /*x*/, int /*p*/) {
    if (expected_lhs <= slack || slack == 0) {
        imp_score = 0;
    } else {
        imp_score = constraint_weight * weighted_lhs / slack;
    }
}

bool Constraint::propagate_lt(std::vector<int>& propagated_assignment) {
    for (size_t i = 0; i < scope.size(); i++) {
        Var* x = scope[i];
        if (!x->has_value()) {
            if (slack < coeff[i]) {
                propagated_assignment.emplace_back(-x->id);
                bool ok = x->set(false, propagated_assignment);
                if (!ok) return false;
            }
        }
    }
    return true;
}

void Constraint::calculate_imp_score_gt() {
    if (accumulated_lhs >= rhs) {
        imp_score = 0;
    } else if (slack == 0) {
        imp_score = 1e9; // Very high score if slack is zero but not satisfied
    } else {
        imp_score = constraint_weight * weighted_lhs / slack;
    }
}

void Constraint::revise_imp_score_gt(Var* /*x*/, int /*p*/) {
    if (accumulated_lhs >= rhs) {
        imp_score = 0;
    } else if (slack == 0) {
        imp_score = 1e9;
    } else {
        imp_score = constraint_weight * weighted_lhs / slack;
    }
}

bool Constraint::propagate_gt(std::vector<int>& propagated_assignment) {
    for (size_t i = 0; i < scope.size(); i++) {
        Var* x = scope[i];
        if (!x->has_value()) {
            if (expected_lhs - coeff[i] < slack) {
                propagated_assignment.emplace_back(x->id);
                bool ok = x->set(true, propagated_assignment);
                if (!ok) return false;
            }
        }
    }
    return true;
}

void Constraint::backpropagate(std::vector<int>& backpropagated_assignment) {
    if (less_than_ct) {
        backpropagate_lt(backpropagated_assignment);
    }
}

void Constraint::backpropagate_lt(std::vector<int>& backpropagated_assignment) {
    backpropagated_assignment.clear();
    for (size_t i = 0; i < scope.size(); i++) {
        Var* x = scope[i];
        if (x->value == 0) {
            bool possible = true;
            for (size_t j = 0; j < x->involving_constraints.size(); j++) {
                Constraint* c = x->involving_constraints[j];
                if (!c->less_than_ct) continue;
                int idx_c = x->involving_constraints_pos[j];
                if (c->slack < coeff[idx_c]) {
                    possible = false;
                    break;
                }
            }
            if (possible) {
                backpropagated_assignment.emplace_back(x->id);
            }
        }
    }
}

std::string Constraint::to_string() {
    std::stringstream ss;
    ss << "c" << id << ": ";
    for (size_t i = 0; i < scope.size(); i++) {
        ss << coeff[i] << "x" << scope[i]->id << " ";
    }
    ss << (less_than_ct ? "<= " : ">= ") << rhs;
    return ss.str();
}

void Constraint::revise_slack() {
    slack = static_cast<double>(rhs) - accumulated_lhs;
}

// --- SelectionConstraintManager Implementation ---

SelectionConstraintManager::SelectionConstraintManager(int nb_vars) : nb_vars(nb_vars), nb_constraints(0) {
    all_vars.resize(nb_vars);
    for (int i = 0; i < nb_vars; i++) {
        all_vars[i] = new Var(i + 1, 0);
    }
}

SelectionConstraintManager::~SelectionConstraintManager() {
    for (Constraint* c : all_constraints) delete c;
    for (Constraint* c : soft_constraints) delete c;
    for (Var* x : all_vars) delete x;
}

void SelectionConstraintManager::set_profit(std::vector<int>& profits) {
    for (int i = 0; i < nb_vars; i++) {
        all_vars[i]->profit = profits[i];
    }
}

void SelectionConstraintManager::update_objective_constraint(int rhs) {
    if (obj_constraint == nullptr) {
        add_objective_constraint(rhs);
    } else {
        if (rhs > obj_constraint->rhs) {
            obj_constraint->rhs = rhs;
            obj_constraint->calculate_imp_score();
        }
    }
}

void SelectionConstraintManager::add_objective_constraint(int rhs) {
    std::vector<int> scope, coeff;
    for (int i = 1; i <= nb_vars; i++) {
        scope.push_back(i);
        coeff.push_back(all_vars[i - 1]->profit);
    }
    add_constraint(scope, coeff, false, rhs); // GT
    obj_constraint = all_constraints.back();
}

void SelectionConstraintManager::add_conflict_constraint(std::vector<int>& scope) {
    std::vector<int> coeff(scope.size(), 1);
    add_constraint(scope, coeff, true, static_cast<int>(scope.size()) - 1); // LT
}

void SelectionConstraintManager::add_disjunctive_constraint(std::vector<int>& scope) {
    std::vector<int> coeff(scope.size(), 1);
    add_constraint(scope, coeff, true, 1); // LT
}

void SelectionConstraintManager::add_knapsack_constraint(std::vector<int>& scope, std::vector<int>& coeff, int rhs, double weight) {
    add_constraint(scope, coeff, true, rhs, weight); // LT
}

void SelectionConstraintManager::add_constraint(std::vector<int>& scope, std::vector<int>& coeff, bool less_than, int rhs, double constraint_weight) {
    std::vector<Var*> ctr_scope;
    for (int i : scope) ctr_scope.emplace_back(all_vars[i - 1]);
    Constraint* ctr = new Constraint(nb_constraints++, coeff, ctr_scope, less_than, rhs);
    ctr->calculate_imp_score(constraint_weight);
    for (size_t i = 0; i < scope.size(); i++) {
        Var* x = all_vars[scope[i] - 1];
        x->involving_constraints.emplace_back(ctr);
        x->involving_constraints_pos.emplace_back(static_cast<int>(i));
        need_revise_score(x);
    }
    all_constraints.emplace_back(ctr);
}

void SelectionConstraintManager::add_soft_conflict(std::vector<int>& scope) {
    std::vector<int> coeff(scope.size(), 1);
    add_soft_constraint(scope, coeff, true, static_cast<int>(scope.size()) - 1);
}

void SelectionConstraintManager::add_soft_clause(std::vector<int>& scope) {
    std::vector<int> coeff(scope.size(), 1);
    add_soft_constraint(scope, coeff, false, 1);
}

void SelectionConstraintManager::add_soft_constraint(std::vector<int>& scope, std::vector<int>& coeff, bool less_than, int rhs, double constraint_weight) {
    std::vector<Var*> ctr_scope;
    for (int i : scope) ctr_scope.emplace_back(all_vars[i - 1]);
    Constraint* ctr = new Constraint(-1, coeff, ctr_scope, less_than, rhs);
    ctr->calculate_imp_score(constraint_weight);
    for (size_t i = 0; i < scope.size(); i++) {
        Var* x = all_vars[scope[i] - 1];
        x->involving_constraints.emplace_back(ctr);
        x->involving_constraints_pos.emplace_back(static_cast<int>(i));
        need_revise_score(x);
    }
    soft_constraints.emplace_back(ctr);
}

void SelectionConstraintManager::maintain_soft_constraint(int soft_tenure) {
    if (soft_constraints.size() == static_cast<size_t>(soft_tenure)) {
        Constraint* c = soft_constraints.front();
        soft_constraints.erase(soft_constraints.begin());
        delete_constraint(c);
    }
}

void SelectionConstraintManager::delete_constraint(Constraint* c) {
    for (Var* x : c->scope) {
        auto it = std::find(x->involving_constraints.begin(), x->involving_constraints.end(), c);
        if (it != x->involving_constraints.end()) {
            int index = static_cast<int>(std::distance(x->involving_constraints.begin(), it));
            x->involving_constraints.erase(it);
            x->involving_constraints_pos.erase(x->involving_constraints_pos.begin() + index);
        }
        need_revise_score(x);
    }
    delete c;
}

bool SelectionConstraintManager::is_allowed(int k, bool v) {
    Var* x = all_vars[k - 1];
    return v ? x->value != 0 : x->value != 1;
}

bool SelectionConstraintManager::assign(int k, bool v) {
    Var* x = all_vars[k - 1];
    propagation_queue.clear();
    if (!x->has_value()) {
        return x->set(v, propagation_queue);
    }
    return true;
}

bool SelectionConstraintManager::assign(std::vector<int> positive_segment) {
    for (int k : positive_segment) {
        if (!assign(k, true)) return false;
    }
    return true;
}

void SelectionConstraintManager::unassign(int k) {
    Var* x = all_vars[k - 1];
    if (!x->has_value()) return;
    x->reset();
    for (auto& c : x->involving_constraints) {
        need_revise_score(c);
    }
}

void SelectionConstraintManager::unassign(std::vector<int> free_segment) {
    for (int k : free_segment) unassign(k);
    revise_constraints();
    revise_objective();
}

void SelectionConstraintManager::relax(std::vector<int> free_segment) {
    for (int i = 1; i <= nb_vars; i++) {
        if (all_vars[i - 1]->value == 0) unassign(i);
    }
    for (int k : free_segment) unassign(k);
    revise_constraints();
    revise_objective();
}

void SelectionConstraintManager::undo_last_decision(int k) {
    for (int i : propagation_queue) unassign(std::abs(i));
    unassign(k);
    propagation_queue.clear();
    for (Constraint* c : all_constraints) c->calculate_imp_score();
    for (Constraint* c : soft_constraints) c->calculate_imp_score();
}

void SelectionConstraintManager::reset_assignment() {
    for (int i = 1; i <= nb_vars; i++) all_vars[i - 1]->reset();
    for (Constraint* c : all_constraints) c->calculate_imp_score();
    for (Constraint* c : soft_constraints) c->calculate_imp_score();
}

bool SelectionConstraintManager::has_next_decision() {
    update_decision_queue();
    return !next_decision_queue.empty();
}

int SelectionConstraintManager::get_next_decision(double greediness) {
    std::vector<selection> candidates;
    while (!next_decision_queue.empty()) {
        selection sel = next_decision_queue.top();
        next_decision_queue.pop();
        if (!all_vars[sel.second - 1]->has_value()) {
            candidates.emplace_back(sel);
        }
    }
    if (candidates.empty()) return 0;
    if (greediness == 1.0) return candidates.front().second;

    double max_val = candidates.front().first;
    double min_val = candidates.back().first;
    double threshold = min_val + (max_val - min_val) * greediness;
    size_t rcl_size = 1;
    for (size_t i = 0; i < candidates.size(); i++) {
        if (candidates[i].first < threshold) {
            rcl_size = i;
            break;
        }
    }
    if (rcl_size == 0) rcl_size = 1;
    return candidates[std::rand() % rcl_size].second;
}

void SelectionConstraintManager::update_decision_queue() {
    clear_decision_queue();
    for (Var* x : all_vars) {
        if (x->has_value()) continue;
        next_decision_queue.push({x->calculate_imp_score(), x->id});
    }
}

bool SelectionConstraintManager::has_new_propagation() {
    return !propagation_queue.empty();
}

int SelectionConstraintManager::get_next_propagation() {
    if (propagation_queue.empty()) return 0;
    int k = propagation_queue.front();
    propagation_queue.erase(propagation_queue.begin());
    return k;
}

void SelectionConstraintManager::clear_decision_queue() {
    while (!next_decision_queue.empty()) next_decision_queue.pop();
}

void SelectionConstraintManager::need_revise_score(Constraint* c) {
    if (!c->less_than_ct) return;
    if (!c->in_revision_queue) {
        constraint_revision_queue.push(c);
        c->in_revision_queue = true;
    }
}

void SelectionConstraintManager::need_revise_score(Var* x) {
    if (!x->in_revision_queue) {
        var_revision_queue.push(x);
        x->in_revision_queue = true;
    }
}

void SelectionConstraintManager::revise_constraints() {
    while (!constraint_revision_queue.empty()) {
        Constraint* c = constraint_revision_queue.front();
        c->in_revision_queue = false;
        constraint_revision_queue.pop();
        c->calculate_imp_score();
        std::vector<int> unassign_vars;
        c->backpropagate(unassign_vars);
        for (int i : unassign_vars) unassign(i);
    }
}

void SelectionConstraintManager::revise_objective() {
    if (obj_constraint) obj_constraint->calculate_imp_score();
}

void SelectionConstraintManager::revise_vars() {
    while (!var_revision_queue.empty()) {
        Var* x = var_revision_queue.front();
        x->in_revision_queue = false;
        var_revision_queue.pop();
        if (!x->has_value()) {
            next_decision_queue.push({x->calculate_imp_score(), x->id});
        }
    }
}

int SelectionConstraintManager::get_size() const {
    return static_cast<int>(all_constraints.size() + soft_constraints.size());
}

Var* SelectionConstraintManager::get_var(int i) { return all_vars[i - 1]; }
int SelectionConstraintManager::get_profit(int i) { return all_vars[i - 1]->profit; }
Constraint* SelectionConstraintManager::get_constraint(int i) { return all_constraints[i]; }

void SelectionConstraintManager::set_seed(int seed) { std::srand(seed); }

void SelectionConstraintManager::show_all_vars() {
    for (Var* x : all_vars) {
        std::cout << "x" << x->id << "=" << (x->has_value() ? std::to_string(x->value) : "?") << "\tR=" << x->profit << std::endl;
    }
}

void SelectionConstraintManager::show_all_constraints() {
    for (Constraint* c : all_constraints) std::cout << c->to_string() << std::endl;
}

// ... remaining methods (estimate_score, reset_constraint_slack, try_select) ...
// (I'll implement them based on the earlier read lines)

double SelectionConstraintManager::estimate_score(Var* x, std::vector<Var*> free_vars, int nb_simulations) {
    double best_gain = 0;
    if (free_vars.size() <= 1) return 0;

    for (int i = 0; i < nb_simulations; i++) {
        double gain = 0;
        reset_constraint_slack();
        
        std::vector<double> noisy_profits;
        for (Var* y : all_vars) noisy_profits.push_back(static_cast<double>(std::rand() % (y->profit + 1)));

        auto sorted_free = free_vars;
        std::sort(sorted_free.begin(), sorted_free.end(), [&noisy_profits](Var* a, Var* b) {
            return noisy_profits[a->id-1] > noisy_profits[b->id-1];
        });

        if (!try_select(x)) continue;
        gain += noisy_profits[x->id - 1];

        for (Var* y : sorted_free) {
            if (y->id == x->id) continue;
            if (try_select(y)) gain += noisy_profits[y->id - 1];
        }
        if (gain > best_gain) best_gain = gain;
    }
    return best_gain;
}

void SelectionConstraintManager::reset_constraint_slack() {
    for (Constraint* c : all_constraints) c->revise_slack();
    for (Constraint* c : soft_constraints) c->revise_slack();
}

bool SelectionConstraintManager::try_select(Var* x) {
    std::vector<std::pair<Constraint*, int>> modified;
    bool ok = true;
    for (size_t i = 0; i < x->involving_constraints.size(); i++) {
        Constraint* c = x->involving_constraints[i];
        if (!c->less_than_ct) continue;
        int coeff = c->coeff[x->involving_constraints_pos[i]];
        c->slack -= coeff;
        modified.push_back({c, coeff});
        if (c->slack < 0) { ok = false; break; }
    }
    if (!ok) {
        for (auto& m : modified) m.first->slack += m.second;
    }
    return ok;
}

void SelectionConstraintManager::update_decision_queue_by_simulation(int nb_simulations) {
    clear_decision_queue();
    std::vector<Var*> free_vars;
    for (Var* x : all_vars) if (!x->has_value()) free_vars.push_back(x);
    for (Var* x : free_vars) {
        next_decision_queue.push({estimate_score(x, free_vars, nb_simulations), x->id});
    }
}

} // namespace oplib::knowledge_base
