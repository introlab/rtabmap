#!/usr/bin/env python3
"""Generate the Doxygen "Parameter reference" page from Parameters.h.

The parameters are declared with the RTABMAP_PARAM* macros, so Doxygen only
ever sees the generated accessors -- one entry per parameter, spread over a
1900-member class page. This script reads the declarations instead and emits a
Markdown page with one table per group, giving the key, type, default value and
description side by side.

Handles what the declarations actually use:
  - RTABMAP_PARAM, RTABMAP_PARAM_STR and RTABMAP_PARAM_COND
  - declarations spanning several lines
  - descriptions built with uFormat("... %s ...", kOther().c_str(), ...), whose
    placeholders are resolved to the referenced parameter keys and linked
  - the same key declared several times under different #if branches (build
    options), whose defaults are all reported with their condition

Usage: generate_parameters_page.py --input Parameters.h --output parameters.md
"""

import argparse
import os
import re
import sys

MACROS = ('RTABMAP_PARAM_COND', 'RTABMAP_PARAM_STR', 'RTABMAP_PARAM')

# Groups are emitted in the order they first appear in Parameters.h (which
# follows the pipeline), with a short blurb where the prefix is not obvious.
GROUP_BLURBS = {
    'Rtabmap': 'Top-level loop closure detection and map management.',
    'Mem': 'Memory management: what is kept in STM/WM, what is transferred to LTM.',
    'Kp': 'Bag-of-words dictionary used for global loop closure detection.',
    'Vis': 'Visual registration (feature extraction, matching and PnP).',
    'Icp': 'Geometric registration by iterative closest point.',
    'Reg': 'Registration strategy shared by loop closures and proximity detection.',
    'RGBD': 'Metric SLAM: graph, proximity detection and localization.',
    'Grid': 'Local occupancy grid generation from each node.',
    'GridGlobal': 'Assembly of the local grids into the global map.',
    'Optimizer': 'Graph optimization back-end.',
    'Marker': 'Fiducial marker (ArUco/AprilTag) detection and landmarks.',
    'Odom': 'Odometry front-end shared settings.',
    'Bayes': 'Bayes filter used for loop closure hypotheses.',
    'Db': 'Database (long-term memory) storage.',
    'Stereo': 'Stereo correspondence.',
}


def negate(condition):
    """Negation of a #if condition, kept readable for the generated page."""
    if condition.startswith('!') and '&&' not in condition and '||' not in condition:
        return condition[1:]
    if re.match(r'^defined\([A-Za-z0-9_]+\)$', condition):
        return '!' + condition
    return '!(%s)' % condition


class Param(object):
    def __init__(self, prefix, name, type_, default, description, branch):
        self.prefix = prefix
        self.name = name
        self.type = type_
        self.description = description
        # (default, branch) pairs: more than one when the parameter is declared
        # in several #if branches. `branch` is the innermost condition only:
        # the branches are read in order, first match wins, so the enclosing
        # negations would only repeat what the previous rows already said.
        self.defaults = [(default, branch)]

    @property
    def key(self):
        return '%s/%s' % (self.prefix, self.name)

    @property
    def accessor(self):
        return 'k%s%s' % (self.prefix, self.name)

    @property
    def anchor(self):
        """Doxygen anchor of this parameter's row, for in-page links."""
        return 'param_%s%s' % (self.prefix, self.name)


def split_args(text):
    """Split a macro argument list on top-level commas."""
    args, depth, current, i, in_string = [], 0, [], 0, False
    while i < len(text):
        c = text[i]
        if in_string:
            if c == '\\':
                current.append(text[i:i + 2])
                i += 2
                continue
            in_string = c != '"'
        elif c == '"':
            in_string = True
        elif c in '([':
            depth += 1
        elif c in ')]':
            depth -= 1
        elif c == ',' and depth == 0:
            args.append(''.join(current).strip())
            current = []
            i += 1
            continue
        current.append(c)
        i += 1
    args.append(''.join(current).strip())
    return args


def unescape(literal):
    """Concatenate adjacent C string literals and undo their escapes."""
    out = []
    for chunk in re.findall(r'"((?:[^"\\]|\\.)*)"', literal):
        out.append(chunk.replace('\\"', '"').replace('\\n', ' ')
                        .replace('\\t', ' ').replace('\\\\', '\\'))
    return ''.join(out) if out else literal.strip()


def parse(path):
    """Return the parameters of Parameters.h, in declaration order."""
    with open(path, newline='') as f:
        lines = f.read().replace('\r\n', '\n').split('\n')

    params, order = {}, []
    conditions = []          # #if nesting, innermost last; None marks an #else
    statement, collecting = '', False
    in_class = False

    for line in lines:
        stripped = line.strip()

        # Everything before the class (include guard, macro definitions) would
        # only add noise to the conditions.
        if not in_class:
            in_class = stripped.startswith('class ') and 'Parameters' in stripped
            continue

        if not collecting and stripped.startswith('#'):
            directive = re.match(r'#\s*(ifdef|ifndef|if|elif|else|endif)\b\s*(.*)', stripped)
            if directive:
                kind, expr = directive.group(1), directive.group(2).strip()
                if kind in ('ifdef', 'ifndef', 'if'):
                    if kind == 'ifdef':
                        expr = 'defined(%s)' % expr
                    elif kind == 'ifndef':
                        expr = '!defined(%s)' % expr
                    conditions.append(expr)
                elif kind in ('elif', 'else') and conditions:
                    conditions[-1] = None if kind == 'else' else expr
                elif kind == 'endif' and conditions:
                    conditions.pop()
            continue

        if not collecting:
            if not re.match(r'\s*RTABMAP_PARAM', line):
                continue
            statement, collecting = line.strip(), True
        else:
            statement += ' ' + stripped

        if statement.count('(') != statement.count(')'):
            continue                     # declaration continues on the next line
        collecting = False

        macro = next(m for m in MACROS if statement.startswith(m))
        args = split_args(statement[len(macro) + 1:statement.rindex(')')])

        prefix, name = args[0], args[1]
        if macro == 'RTABMAP_PARAM_STR':
            type_, default, description = 'string', unescape(args[2]), args[3]
        elif macro == 'RTABMAP_PARAM_COND':
            type_, description = args[2], args[6]
            default = '%s if `%s`, %s otherwise' % (args[4], args[3], args[5])
        else:
            type_, default, description = args[2], args[3], args[4]

        branch = conditions[-1] if conditions else ''
        key = '%s/%s' % (prefix, name)
        if key in params:
            params[key].defaults.append((default, branch))
        else:
            params[key] = Param(prefix, name, type_, default, description, branch)
            order.append(key)

    return [params[k] for k in order]


def accessor_link(param):
    """A link to the C++ accessor, labelled with the parameter key."""
    return '@ref rtabmap::Parameters::%s() "%s"' % (param.accessor, param.key)


def key_cell(param):
    """The key column: the row's own anchor, then the link to the accessor."""
    return '@anchor %s %s' % (param.anchor, accessor_link(param))


def page_link(param):
    """A link to the parameter's own row on this page."""
    return '@ref %s "%s"' % (param.anchor, param.key)


def resolve_description(param, by_accessor):
    """Render the description, resolving uFormat() placeholders to key links."""
    text = param.description.strip()
    if text.startswith('uFormat('):
        args = split_args(text[len('uFormat('):text.rindex(')')])
        text = unescape(args[0])
        for arg in args[1:]:
            match = re.search(r'\bk([A-Za-z0-9]+)\(\)', arg)
            replacement = arg.strip()
            if match:
                other = by_accessor.get('k' + match.group(1))
                replacement = page_link(other) if other else match.group(1)
            if '"%s"' in text:
                text = text.replace('"%s"', replacement, 1)
            else:
                text = text.replace('%s', replacement, 1)
    else:
        text = unescape(text)
    return escape_cell(text)


def escape_cell(text):
    """A table cell cannot contain a raw '|', and '<' would open an HTML tag."""
    return text.replace('|', '\\|').replace('<', '&lt;').replace('>', '&gt;')


def format_defaults(param):
    def code(value):
        value = value.strip()
        return '`%s`' % value if value else '`""`'

    # Consecutive branches giving the same value are merged, so that a value
    # that only changes in the last #else does not repeat three times.
    merged = []
    for value, branch in param.defaults:
        if merged and merged[-1][0] == value:
            merged[-1][1].append(branch)
        else:
            merged.append([value, [branch]])

    if len(merged) == 1:
        value, branches = merged[0]
        if not any(branches) or None in branches:
            return code(value)
        return '%s with `%s`' % (code(value), escape_cell(' or '.join(branches)))

    parts = []
    for value, branches in merged:
        if None in branches or not any(branches):
            parts.append('%s otherwise' % code(value))
        else:
            parts.append('%s with `%s`' % (code(value), escape_cell(' or '.join(branches))))
    return '<br>'.join(parts)


def render(params, source_name):
    groups, order = {}, []
    for p in params:
        groups.setdefault(p.prefix, []).append(p)
        if p.prefix not in order:
            order.append(p.prefix)

    by_accessor = {p.accessor: p for p in params}

    out = []
    out.append('Parameter reference {#parameters}')
    out.append('===================')
    out.append('')
    out.append('Every setting in RTAB-Map is a `Group/Name` string key with a string value,')
    out.append('collected in a rtabmap::ParametersMap. The %d parameters below are declared in'
               % len(params))
    out.append('`%s`; the same keys are used by rtabmap::Rtabmap and rtabmap::Odometry, by the'
               % source_name)
    out.append('applications, by the `--Param Group/Name value` command-line arguments of the')
    out.append('tools and by the ROS wrappers, so a setting found here applies everywhere.')
    out.append('')
    out.append('~~~{.cpp}')
    out.append('rtabmap::ParametersMap parameters;')
    out.append('parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemSTMSize(), "20"));')
    out.append('rtabmap.init(parameters, "map.db");')
    out.append('~~~')
    out.append('')
    out.append('Each key links to its accessor on rtabmap::Parameters, which is also how the')
    out.append('key is spelled in code (`Parameters::kMemSTMSize()` for `Mem/STMSize`).')
    out.append('Defaults given with a condition depend on how RTAB-Map was built; call')
    out.append('rtabmap::Parameters::getDefaultParameters() to read the values of the build in use.')
    out.append('')
    out.append('**Groups:** ' + ', '.join(
        '@ref parameters_%s "%s"' % (p, p) for p in order))
    out.append('')

    for prefix in order:
        # @anchor rather than a "{#id}" heading: the latter would turn the
        # heading into a section, which lands in the navigation tree whatever
        # TOC_INCLUDE_HEADINGS says.
        out.append('@anchor parameters_%s' % prefix)
        out.append(prefix)
        out.append('-' * len(prefix))
        out.append('')
        if prefix in GROUP_BLURBS:
            out.append(GROUP_BLURBS[prefix])
            out.append('')
        out.append('| Key | Type | Default | Description |')
        out.append('| --- | ---- | ------- | ----------- |')
        for p in groups[prefix]:
            out.append('| %s | %s | %s | %s |' % (
                key_cell(p), p.type, format_defaults(p), resolve_description(p, by_accessor)))
        out.append('')

    return '\n'.join(out) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, help='path to Parameters.h')
    parser.add_argument('--output', required=True, help='Markdown page to write')
    args = parser.parse_args()

    params = parse(args.input)
    if not params:
        sys.stderr.write('error: no parameter found in %s\n' % args.input)
        return 1

    directory = os.path.dirname(os.path.abspath(args.output))
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with open(args.output, 'w') as f:
        f.write(render(params, os.path.basename(args.input)))
    sys.stderr.write('%s: %d parameters\n' % (args.output, len(params)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
