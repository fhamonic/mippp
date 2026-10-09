// Expected NOT to compile: an unconstrained set_status forwarding a tag the
// variant does not list reaches handle_status_table's deleted overload.
#include <variant>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

using namespace mippp;
namespace bs = mippp::basis_status;
using variable = model_variable<int, double>;
using var_status = std::variant<bs::nonbasic_at_lower_bound, bs::basic,
                                bs::nonbasic_at_upper_bound, bs::nonbasic_free>;

struct snap {
    detail::handle_status_table<var_status> vars{1};
    template <typename Tag>
    void set_status(variable v, Tag t) {
        vars.set(v.uid(), t);
    }
};

int main() {
    snap s;
    s.set_status(variable(0), bs::nonbasic_fixed{});
}
