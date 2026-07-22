---
name: diagnostics-ux-review
description: Audit the library's friendly compile-time diagnostics end-to-end — trigger each one, verify the message fires before raw template noise, and prove its recommended remedy actually compiles. Use after changing concepts/asserts/operators, or when a user reports a confusing compile error.
---

# Purpose

In a template library, compile errors are the user interface: most users
meet `static_assert("MIP++: ...")` messages long before they read docs.
This skill audits those diagnostics like UI — every one deliberately
triggered, its noise measured, its remedy verified — instead of trusting
that a message written once still fires, still fires *first*, and still
gives advice that compiles.

# When to use

- After adding/renaming/reordering concepts, `assert_*` helpers, or the
  operators that call them.
- After changing what an error message's remedy refers to (e.g.
  `materialize`, `std::move`, "build over a named range").
- When a user reports an incomprehensible template error — the audit tells
  you whether a diagnostic is missing or merely buried.
- Periodically, since diagnostics have no tests that fail when they rot.

# The diagnostic system (invariants)

- Friendly diagnostics are `consteval void assert_*()` helpers in `detail`,
  with messages prefixed `"MIP++: "`. Inventory them with:
  `grep -rn "consteval void assert_" include/` and
  `grep -rn "MIP++:" include/`.
- Operators call the assert **as the first statements of the body**, before
  the `return` that instantiates the view class — statements instantiate in
  order, so the friendly message prints before any constraint failure from
  the class-level `requires`. That ordering is load-bearing: never move the
  assert after the return expression, and never delete the class `requires`
  in its favor (see `cpp-template-library-review`).
- The docs quote these messages in a troubleshooting table
  (`docs/reference/expression-layer.md`); message edits propagate there
  (see `api-doc-sync`).
- The build adds `-fconcepts-diagnostics-depth=30` for readable concept
  failures; scratch audits should compile with the same flag.

# Workflow: full audit

1. **Inventory**: list every assert helper, its message, the operators that
   call it, and the remedy each message promises. A helper no operator
   calls, or an operator consuming operands without the matching assert, is
   finding #1.
2. **Write one trigger TU per diagnostic** in the scratchpad — the minimal
   user code that commits the mistake the message describes (single-use
   lvalue reused, `xsum` result squared, mixed scalar types...). Compile
   with the quick syntax-check command.
3. **For each trigger, verify four properties**:
   - [ ] the `MIP++:` message appears;
   - [ ] it appears at the **top** of stderr — count the error lines before
         it; more than a screenful of template noise first means the assert
         fires too late or a hard error preempts it;
   - [ ] it is the *right* message (compatible-scalars mistakes must not
         land in the single-use message);
   - [ ] no second, contradictory friendly message follows.
4. **Verify every remedy compiles.** Take the failing TU and apply each fix
   the message proposes, literally as worded. Each variant must compile and
   behave. A remedy that itself fails is a real bug class (a `materialize`
   escape hatch that produces a type frozen against the next step was found
   exactly this way).
5. **Cover the mirrors.** Trigger via lvalue and rvalue operands, both
   operand orders, and through every operator sharing the assert (`+`, `-`,
   `xsum`, `+=`...). An assert present in `operator+` but missing in
   `operator+=` was a real gap.
6. **Verify the silent path**: the nearest *correct* usages must compile
   with zero warnings under `-Wall` — a diagnostic that fires on valid code
   is worse than noise.
7. Report per diagnostic: fires? position in stderr? remedy valid? gaps.
   Fix message text, assert placement, and missing call sites; sync docs.

# Message style rules (for new or edited diagnostics)

- [ ] Prefix `"MIP++: "` — greppable, and users can search it.
- [ ] Sentence 1: the *situation* in user terms ("this expression owns its
      term range ... and is single-use"), not the failed metafunction.
- [ ] Then the remedies, in preference order, naming exact spellings the
      user can type (`std::move`, `materialize(e)`, "build it over a named
      range").
- [ ] State the *reason* when it changes what the user should do ("a
      product traverses each operand once per term of the other").
- [ ] No internal jargon that has no doc anchor; every named function in a
      message must exist (grep after renames — messages don't fail builds).

# Quality checklist

- [ ] Every assert helper has ≥1 trigger TU exercised in the audit.
- [ ] Every message remedy compiled verbatim.
- [ ] Every consuming operator calls the matching assert (diff the operator
      list against the assert call sites).
- [ ] Docs troubleshooting table matches current message text (grep a
      distinctive fragment in both trees).
- [ ] Audit run on the strictest available compiler set — GCC accepts some
      ISO-invalid code as extensions; a Clang pass catches what GCC hides.

# Common mistakes to avoid

- Testing that the *code* fails to compile without checking *which* error
  the user actually sees first.
- Editing a message's advice without recompiling the advice.
- Renaming an API and leaving its old name inside message strings and the
  docs table — neither is compiled.
- Adding a new consuming operation (or accessor) without wiring the assert,
  leaving raw `deleted function` errors as its diagnostic.
- Auditing only rvalue happy-path triggers; the lvalue/const mirrors take
  different overload paths and can bypass the assert.
