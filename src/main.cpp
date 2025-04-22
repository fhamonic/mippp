#include <iostream>

#include "mippp/constraints/linear_constraint_operators.hpp"
#include "mippp/expressions/linear_expression_operators.hpp"
#include "mippp/mip_model.hpp"

using namespace fhamonic::mippp;

struct Note {
    int id;
    std::string name;
    double freq;
};

constexpr int num_harmonics = 7;

struct Harmonic {
    int num;
    std::string name;
    double freq;
};

int main() {
    using model = mip_model<linked_scip_traits>;
    model builder(model::opt_sense::max);

    std::vector<Note> notes = {{0, "C4", 261.63},  {1, "D4", 293.66},
                               {2, "Eb4", 311.13}, {3, "F4", 349.23},
                               {4, "G4", 392},     {5, "A4", 440},
                               {6, "Bb4", 493.88}, {7, "C5", 523.25}};

    std::vector<std::vector<Harmonic>> notes_harmonics(notes.size());

    auto positions =
        std::ranges::views::iota(0, static_cast<int>(notes.size()));

    for(auto && note : notes) {
        for(int i = 0; i < num_harmonics; ++i)
            notes_harmonics[note.id].emplace_back(
                Harmonic{i + 1, note.name + "_" + std::to_string(i + 1),
                         note.freq / (2 * i + 1)});
    }

    auto note_dist = [&](auto && note1, auto && note2) {
        double dist = 0;
        for(auto && harm1 : notes_harmonics[note1.id]) {
            double min_harm_dist = std::numeric_limits<double>::max();
            for(auto && harm2 : notes_harmonics[note2.id]) {
                min_harm_dist = std::min(
                    min_harm_dist,
                    std::abs(std::log(harm1.freq) - std::log(harm2.freq)));
            }
            // dist += min_harm_dist;
            dist += min_harm_dist / harm1.num;
        }
        return std::sqrt(dist);
    };

    auto pos_dist = [&](auto && pos1, auto && pos2) {
        if(pos2 < pos1) std::swap(pos1, pos2);
        int dist = std::min(pos2 - pos1, 8 + pos1 - pos2);
        if(dist == 1) return 10.0;
        if(dist == 2) return 60.0;
        if(dist == 3) return 100.0;
        if(dist == 4) return 130.0;
        return 0.0;
    };

    auto note_position_vars = builder.add_variables(
        notes.size() * notes.size(),
        [n = notes.size()](const Note & note, const int position) {
            return note.id * n + position;
        },
        {.upper_bound = 1, .type = model::var_category::binary});

    for(auto && note : notes) {
        builder.add_constraint(xsum(positions, [&](auto && pos) {
                                   return note_position_vars(note, pos);
                               }) == 1);
    }
    for(auto && pos : positions) {
        builder.add_constraint(xsum(notes, [&](auto && note) {
                                   return note_position_vars(note, pos);
                               }) == 1);
    }

    auto z_vars = builder.add_variables(
        notes.size() * notes.size() * notes.size() * notes.size(),
        [n = notes.size()](const Note & note1, const int pos1,
                           const Note & note2, const int pos2) {
            return (note1.id * n + pos1) * (n * n) + (note2.id * n + pos2);
        },
        {.upper_bound = 1, .type = model::var_category::continuous});

    for(auto && note1 : notes) {
        for(auto && pos1 : positions) {
            for(auto && note2 : notes) {
                for(auto && pos2 : positions) {
                    if(note1.id == note2.id) continue;
                    if(pos1 == pos2) continue;

                    builder.add_constraint(z_vars(note1, pos1, note2, pos2) <=
                                           note_position_vars(note1, pos1));
                    builder.add_constraint(z_vars(note1, pos1, note2, pos2) <=
                                           note_position_vars(note2, pos2));
                    builder.add_constraint(z_vars(note1, pos1, note2, pos2) >=
                                           note_position_vars(note1, pos1) +
                                               note_position_vars(note2, pos2) -
                                               1);
                    builder.add_to_objective(
                        note_dist(note1, note2) /
                        (pos_dist(pos1, pos2) * pos_dist(pos1, pos2)) *
                        z_vars(note1, pos1, note2, pos2));
                }
            }
        }
    }

    builder.add_constraint(note_position_vars(notes[0], 0) == 1);
    builder.add_constraint(note_position_vars(notes[7], 4) == 1);

    std::cout << builder << std::endl;

    auto solver_model = builder.build();
    solver_model.solve();

    auto solution = solver_model.get_solution();
    for(auto && note : notes) {
        for(auto && pos : positions) {
            if(solution[note_position_vars(note, pos).id()] > 0) {
                std::cout << note.name << " " << pos << std::endl;
            }
        }
    }

    return EXIT_SUCCESS;
}