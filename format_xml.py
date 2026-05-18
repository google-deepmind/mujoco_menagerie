# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# /// script
# requires-python = ">=3.10"
# dependencies = ["lxml"]
# ///
"""Format MJCF XML files in the style enforced by CONTRIBUTING.md.

Rules (matching the Red Hat XML extension's defaults for this repo):
  - 2-space indent
  - Double-quoted attribute values
  - Self-closing empty elements: <foo/>  (no space before the slash)
  - Wrap a tag's attributes onto multiple lines when it exceeds 120 cols
  - Preserve user-written blank lines between sibling elements
  - Preserve comments

Usage:
  uv run scripts/format_xml.py --check <path>...  # exit 1 if any file is
                                                  # not formatted
  uv run scripts/format_xml.py --write <path>...  # rewrite files in place
  uv run scripts/format_xml.py <path>             # print formatted to stdout
"""

from __future__ import annotations

import argparse
import pathlib
import sys

from lxml import etree

INDENT = '  '
MAX_WIDTH = 120


def _escape_attr(value: str) -> str:
  # Collapse runs of whitespace (incl. newlines) to a single space and strip.
  # XML attribute-value normalization already converts newlines to spaces at
  # parse time, so multi-line values authored for readability cannot be
  # recovered; collapsing yields a clean canonical form.
  value = ' '.join(value.split())
  return (
    value.replace('&', '&amp;')
    .replace('<', '&lt;')
    .replace('>', '&gt;')
    .replace('"', '&quot;')
  )


def _attr_str(attrs) -> str:
  return ' '.join(f'{k}="{_escape_attr(v)}"' for k, v in attrs.items())


def _blank_lines_between(text_or_tail: str | None) -> int:
  """Returns the number of blank lines the user wrote in this whitespace."""
  if not text_or_tail:
    return 0
  # A blank line shows up as 2+ consecutive newlines in the whitespace text.
  return max(0, text_or_tail.count('\n') - 1)


def _format_open_tag(
  tag: str, attrs, depth: int, self_close: bool
) -> list[str]:
  """Format an opening (or self-closing) tag, wrapping at MAX_WIDTH.

  Wrap policy: greedily fill the opening line; spill subsequent attributes
  onto continuation lines indented by (depth + 1) * INDENT.
  """
  prefix = INDENT * depth
  cont = INDENT * (depth + 1)
  suffix = '/>' if self_close else '>'

  if not attrs:
    return [f'{prefix}<{tag}{suffix}']

  items = list(attrs.items())
  one_line = f'{prefix}<{tag} {_attr_str(attrs)}{suffix}'
  if len(one_line) <= MAX_WIDTH:
    return [one_line]

  lines: list[str] = []
  current = f'{prefix}<{tag}'
  for i, (k, v) in enumerate(items):
    attr = f'{k}="{_escape_attr(v)}"'
    is_last = i == len(items) - 1
    candidate = f'{current} {attr}'
    limit = MAX_WIDTH - (len(suffix) if is_last else 0)
    if len(candidate) <= limit:
      current = candidate
    else:
      lines.append(current)
      current = f'{cont}{attr}'
  lines.append(current + suffix)
  return lines


def _serialize(node, depth: int, out: list[str]) -> None:
  prefix = INDENT * depth

  # Comments: <!-- ... --> (lxml represents them as etree._Comment).
  if isinstance(node, etree._Comment):
    text = node.text or ''
    out.append(f'{prefix}<!--{text}-->')
    return

  tag = node.tag
  attrs = node.attrib
  children = list(node.iterchildren())
  has_children = bool(children)
  text = (node.text or '').strip()
  has_text = bool(text)

  if not has_children and not has_text:
    out.extend(_format_open_tag(tag, attrs, depth, self_close=True))
    return

  out.extend(_format_open_tag(tag, attrs, depth, self_close=False))

  if has_text and not has_children:
    out.append(f'{prefix}{INDENT}{text}')

  for i, child in enumerate(children):
    if i == 0:
      prev_ws = node.text
    else:
      prev_ws = children[i - 1].tail
    if _blank_lines_between(prev_ws) > 0:
      out.append('')
    _serialize(child, depth + 1, out)

  out.append(f'{prefix}</{tag}>')


def format_xml(source: str) -> str:
  parser = etree.XMLParser(remove_blank_text=False, remove_comments=False)
  root = etree.fromstring(source.encode('utf-8'), parser=parser)
  out: list[str] = []
  _serialize(root, 0, out)
  return '\n'.join(out) + '\n'


def main() -> int:
  ap = argparse.ArgumentParser(description='Format MJCF XML files.')
  mode = ap.add_mutually_exclusive_group()
  mode.add_argument(
    '--check',
    action='store_true',
    help='Exit 1 if any file is not properly formatted.',
  )
  mode.add_argument(
    '--write',
    action='store_true',
    help='Rewrite files in place.',
  )
  ap.add_argument('paths', nargs='+', type=pathlib.Path)
  args = ap.parse_args()

  failed: list[pathlib.Path] = []
  for p in args.paths:
    text = p.read_text()
    formatted = format_xml(text)
    if args.check:
      if text != formatted:
        failed.append(p)
    elif args.write:
      if text != formatted:
        p.write_text(formatted)
    else:
      sys.stdout.write(formatted)

  if args.check and failed:
    print('Not formatted:', file=sys.stderr)
    for p in failed:
      print(f'  {p}', file=sys.stderr)
    return 1
  return 0


if __name__ == '__main__':
  sys.exit(main())
