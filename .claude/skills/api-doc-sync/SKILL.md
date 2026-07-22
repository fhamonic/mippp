---
name: api-doc-sync
description: Keep docs truthful when the library changes — treat every doc sentence as an assertion. Use after renaming identifiers, changing operator/overload semantics, altering error messages, or adding features; also when writing new reference docs.
---

# Purpose

Documentation in this repo makes falsifiable claims: it names identifiers,
states which overloads exist and what they return, quotes static_assert
messages, embeds code snippets, and publishes generated feature tables.
Each of those is an assertion that silently rots when the code moves. This
skill makes doc updates a verification step of every change, not an
afterthought — semantics land as **code + test + doc in one change**.

# When to use

- After any rename, semantic change, new feature, or error-message edit in
  the library headers.
- When writing or reviewing a `docs/reference/*.md` page.
- When a doc claim and the code are suspected to disagree.

# Doc-claim taxonomy

Know what kinds of assertions a page makes; each rots differently and each
has its own check:

| Claim kind | Example | Check |
| --- | --- | --- |
| Identifier names | `detail::forwardable_linear_expression` | grep docs for the old name after any rename |
| Semantic statements | "`zero * s` is still `zero_t`" | grep for the operator/keyword; reread the sentence against the new behavior |
| Lifetime/ownership contracts | "a named operand is only referenced" | must be added when semantics change, with the user-facing consequence stated |
| Code snippets | fenced ```cpp blocks | extract to a scratch TU and syntax-check |
| Error-message quotes | tables mapping messages to fixes | grep a distinctive fragment of the message in both headers and docs |
| Generated artifacts | feature tables from `INSTANTIATE_TEST` lines | regenerate, commit outputs with the change |
| Design rationale | "enforced on the class, so traits do not lie" | verify the mechanism still exists before citing it |

# Workflow: after a library change

1. **Build the grep list** from the diff: every removed/renamed identifier,
   every operator whose semantics changed, distinctive fragments of every
   edited diagnostic message.
2. `grep -rn <term> docs/` for each. Also grep the **concept vocabulary**
   around the change ("copy", "reference", "owns", "noexcept", "fold") —
   prose paraphrases rot even when identifiers don't appear.
3. Update claims **in place**; where behavior is new, add a paragraph that
   states the user-visible contract (what now works, what the lifetime rule
   is, what the escape hatch is) — not the implementation.
4. **Validate snippets by compilation**: copy each affected fenced block
   into a scratch `.cpp` and run the quick syntax check (GCC 15 +
   `-fsyntax-only`, no build system needed — see project memory for the
   exact command). A snippet that needs invented context to compile should
   be rewritten until it doesn't.
5. Regenerate generated artifacts (`make featuers_tables`) if test
   instantiations changed; commit the regenerated files with the change.
6. Ship the doc edit in the same commit as the code and tests.

# Workflow: writing a new reference page

1. **Derive from tests, not from the headers.** The reorganized test files
   read as a requirements narrative; a good reference section follows the
   same order (what the type is → operations → operand requirements →
   escape hatches → special branches) and links the test file as the
   authoritative example set.
2. State contracts, not mechanics: what the user may rely on, what is
   deliberately absent (and why), what the error message will tell them.
   Deliberate omissions ("`s / zero` is intentionally not provided") are
   documentation-worthy decisions.
3. Every code block must be one the syntax-checker accepts verbatim.
4. Prefer generated tables over hand-maintained ones; if a table cannot be
   generated, add a comment in the source it mirrors, pointing at the doc.
5. When documenting a diagnostic, quote the real message text and the real
   fix — then trigger it once in a scratch TU to confirm both.

# Quality checklist

- [ ] `grep -rn` for every old identifier in `docs/` returns nothing.
- [ ] Every semantic sentence touched by the change reread and either
      updated or confirmed.
- [ ] Every affected snippet syntax-checked.
- [ ] New semantics have a stated lifetime/ownership contract, not just a
      feature announcement.
- [ ] Generated tables regenerated and committed if instantiations changed.
- [ ] Doc, code, and tests in the same commit.

# Common mistakes to avoid

- Renaming in code and tests but leaving the doc's inline-code mentions —
  identifiers in prose don't fail the build; only grep catches them.
- Updating the main explanation but missing the secondary mention: the
  troubleshooting table, the "What's next" cross-reference, the docstring
  quoting the old error text.
- Documenting the implementation ("stores a pointer") instead of the
  contract ("the operand must outlive the view, like any `ref_view`").
- Adding a snippet that compiles only with extra context the reader can't
  see.
- Regenerating feature tables locally but committing only the source diff,
  leaving stale images in the docs.
- Writing docs from the headers' internals when the tests already narrate
  the intended usage order.
