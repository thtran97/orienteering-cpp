#pragma once

#include <vector>
#include <queue>
#include <string>
#include <memory>

namespace oplib::knowledge_base {

class Constraint;

/**
 * @brief Represents a variable in the knowledge base.
 */
class Var {
public:
    Var(int id, int profit);
    ~Var() = default;

    bool has_value() const;
    bool set(bool v, std::vector<int>& propagated_assignment);
    void reset();
    double calculate_imp_score();

    int id;
    int profit;
    int value; // -1 for unknown (UKN)
    std::vector<Constraint*> involving_constraints;
    std::vector<int> involving_constraints_pos;
    bool in_revision_queue = false;
};

/**
 * @brief Represents a constraint in the knowledge base.
 */
class Constraint {
public:
    Constraint(int id, std::vector<int>& coeff, std::vector<Var*> scope, bool less_than, int rhs);
    ~Constraint() = default;

    bool is_inconsistent();
    bool is_satisfied();
    void calculate_imp_score(double weighted_coeff = 1.0);
    void revise_imp_score(Var* x, int p, bool is_set);
    bool propagate(std::vector<int>& propagated_assignment);
    void backpropagate(std::vector<int>& backpropagated_assignment);

    std::string to_string();
    void revise_slack();

    int id;
    bool less_than_ct;
    int rhs;
    std::vector<int> coeff;
    std::vector<Var*> scope;
    double constraint_weight;
    
    // Dynamic attributes
    int expected_lhs;
    int accumulated_lhs;
    double weighted_lhs;
    double slack;
    double imp_score;
    bool in_revision_queue = false;

private:
    void calculate_imp_score_lt();
    void calculate_imp_score_gt();
    void revise_imp_score_lt(Var* x, int p);
    void revise_imp_score_gt(Var* x, int p);
    bool propagate_lt(std::vector<int>& propagated_assignment);
    bool propagate_gt(std::vector<int>& propagated_assignment);
    void backpropagate_lt(std::vector<int>& backpropagated_assignment);
};

using selection = std::pair<double, int>;

struct increasing_cmp {
    bool operator()(selection const& a, selection const& b) const {
        return a.first < b.first;
    }
};

struct decreasing_cmp {
    bool operator()(selection const& a, selection const& b) const {
        return a.first > b.first;
    }
};

using selection_pqueue_min = std::priority_queue<selection, std::vector<selection>, increasing_cmp>;
using selection_pqueue_max = std::priority_queue<selection, std::vector<selection>, decreasing_cmp>;

/**
 * @brief Manages selection constraints and variables.
 */
class SelectionConstraintManager {
public:
    SelectionConstraintManager(int nb_vars);
    ~SelectionConstraintManager();

    void set_profit(std::vector<int>& profits);
    void update_objective_constraint(int rhs);
    void add_objective_constraint(int rhs);
    void add_conflict_constraint(std::vector<int>& scope);
    void add_disjunctive_constraint(std::vector<int>& scope);
    void add_knapsack_constraint(std::vector<int>& scope, std::vector<int>& coeff, int rhs, double weight = 1.0);
    
    void add_soft_conflict(std::vector<int>& scope);
    void add_soft_clause(std::vector<int>& scope);
    void maintain_soft_constraint(int soft_tenure);
    
    bool is_allowed(int k, bool v);
    bool assign(int k, bool v);
    bool assign(std::vector<int> positive_segment);
    
    void unassign(std::vector<int> free_segment);
    void unassign(int k);
    void relax(std::vector<int> free_segment);
    void undo_last_decision(int k);
    void reset_assignment();
    
    bool has_next_decision();
    int get_next_decision(double greediness = 1.0);
    bool has_new_propagation();
    int get_next_propagation();

    void revise_constraints();
    void revise_objective();
    void revise_vars();

    void show_all_vars();
    void show_all_constraints();

    int get_size() const;

    void update_decision_queue_by_simulation(int nb_simulations);
    double estimate_score(Var* x, std::vector<Var*> free_vars, int nb_simulations);
    void reset_constraint_slack();
    bool try_select(Var* x);
    void set_seed(int seed);

private:
    int nb_vars;
    int nb_constraints;
    std::vector<Var*> all_vars;
    std::vector<Constraint*> all_constraints;
    Constraint* obj_constraint = nullptr;
    std::vector<Constraint*> soft_constraints;

    selection_pqueue_min next_decision_queue;
    std::vector<int> propagation_queue;
    std::queue<Constraint*> constraint_revision_queue;
    std::queue<Var*> var_revision_queue;

    Var* get_var(int i);
    int get_profit(int i);
    Constraint* get_constraint(int i);

    void add_constraint(std::vector<int>& scope, std::vector<int>& coeff, bool less_than, int rhs, double constraint_weight = 1.0);
    void add_soft_constraint(std::vector<int>& scope, std::vector<int>& coeff, bool less_than, int rhs, double constraint_weight = 1.0);
    void delete_constraint(Constraint* c);

    void clear_decision_queue();
    void update_decision_queue();

    void need_revise_score(Constraint* c);
    void need_revise_score(Var* x);
};

} // namespace oplib::knowledge_base
