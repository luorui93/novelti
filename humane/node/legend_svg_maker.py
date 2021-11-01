#!/usr/bin/python 
import yaml
import sys
import os

usage = """
USAGE:
    ./make_legend.py operator_name

"""

datadir = os.environ['HOME']+"/humane_data"

tmpl1 = """<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<!-- Created with Inkscape (http://www.inkscape.org/) -->

<svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   viewBox="0 0 910 %(height)d"
   id="svg2"
   version="1.1"
   inkscape:version="0.91 r13725"
   sodipodi:docname="1.svg">
  <defs
     id="defs4" />
  <sodipodi:namedview
     id="base"
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1.0"
     inkscape:pageopacity="0.0"
     inkscape:pageshadow="2"
     inkscape:zoom="1.0"
     inkscape:cx="214.11462"
     inkscape:cy="-70.918228"
     inkscape:document-units="px"
     inkscape:current-layer="layer1"
     showgrid="false"
     inkscape:window-width="1155"
     inkscape:window-height="1056"
     inkscape:window-x="65"
     inkscape:window-y="24"
     inkscape:window-maximized="1"
     fit-margin-top="5"
     fit-margin-left="5"
     fit-margin-right="5"
     fit-margin-bottom="5" />
  <metadata
     id="metadata7">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title></dc:title>
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <g
     inkscape:label="Layer 1"
     inkscape:groupmode="layer"
     id="layer1"
     transform="translate(16.706383,10.178289)">
    <text
       xml:space="preserve"
       style="font-style:normal;font-variant:normal;font-weight:bold;font-stretch:normal;font-size:40px;line-height:125%%;font-family:Sans;-inkscape-font-specification:'Sans Bold';letter-spacing:0px;word-spacing:0px;fill:#000000;fill-opacity:1;stroke:none;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       x="3"
       y="37.928871"
       id="text4146"
       sodipodi:linespacing="125%%"><tspan
         sodipodi:role="line"
         id="tspan4148"
         x="3"
         y="37.928871">color name</tspan></text>
    <text
       xml:space="preserve"
       style="font-style:normal;font-variant:normal;font-weight:bold;font-stretch:normal;font-size:40px;line-height:125%%;font-family:Sans;-inkscape-font-specification:'Sans Bold';letter-spacing:0px;word-spacing:0px;fill:#000000;fill-opacity:1;stroke:none;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       x="321.65945"
       y="37.928871"
       id="text4146-3"
       sodipodi:linespacing="125%%"><tspan
         sodipodi:role="line"
         id="tspan4148-6"
         x="321.65945"
         y="37.928871">color</tspan></text>
    <text
       xml:space="preserve"
       style="font-style:normal;font-variant:normal;font-weight:bold;font-stretch:normal;font-size:40px;line-height:125%%;font-family:Sans;-inkscape-font-specification:'Sans Bold';letter-spacing:0px;word-spacing:0px;fill:#000000;fill-opacity:1;stroke:none;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       x="529.88678"
       y="37.928871"
       id="text4146-3-7"
       sodipodi:linespacing="125%%"><tspan
         sodipodi:role="line"
         id="tspan4148-6-5"
         x="529.88678"
         y="37.928871">expression</tspan></text>
    <path
       style="fill:none;fill-rule:evenodd;stroke:#000000;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       d="m 1.0101525,56.351795 772.7666975,0"
       id="path4186"
       inkscape:connector-curvature="0"
       sodipodi:nodetypes="cc" />
"""
tmpl2 = """
    <rect
       style="fill:rgb(%(r)d,%(g)d,%(b)d);fill-opacity:1;stroke:none;stroke-width:0.69999999;stroke-linecap:round;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1"
       id="rect4136"
       width="150"
       height="50"
       x="320"
       y="%(recy)d" />
    <text
       xml:space="preserve"
       style="font-style:normal;font-weight:normal;font-size:40px;line-height:125%%;font-family:Sans;letter-spacing:0px;word-spacing:0px;fill:#000000;fill-opacity:1;stroke:none;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       x="3"
       y="%(y)d"
       id="text4138"
       sodipodi:linespacing="125%%"><tspan
         sodipodi:role="line"
         id="tspan4140"
         x="3"
         y="%(y)d">%(name)s</tspan></text>
    <text
       xml:space="preserve"
       style="font-style:normal;font-weight:normal;font-size:40px;line-height:125%%;font-family:Sans;letter-spacing:0px;word-spacing:0px;fill:#000000;fill-opacity:1;stroke:none;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       x="530"
       y="%(y)d"
       id="text4142"
       sodipodi:linespacing="125%%"><tspan
         sodipodi:role="line"
         id="tspan4144"
         x="530"
         y="%(y)d">%(expr)s</tspan></text>  
"""
tmpl3 = """
  </g>
</svg>
"""


dy = 70


if len(sys.argv)!=2:
    print usage
    exit(1)


color_rgbs  = [(255,255,0), (0,0,255), (0,255,0), (255,0,0)]
color_names = ["yellow", "blue", "green", "red"]
expressions = ["clench", "smirk left", "smirk right", "raise eyebrow"]

with open(datadir + "/emotiv_cmd_mapping/" + sys.argv[1]+ "_mapping.yaml") as f:
    mappings = {v: k for k, v in yaml.load(f)["key_mappings"].iteritems()}
    with open("/tmp/legend_colors.svg", "w") as text_file:
        text_file.write(tmpl1 % {"height" : 110 + dy*len(color_rgbs) })
        for k in sorted(mappings.keys()):
            text_file.write(tmpl2 % {
                "y":    110+k*dy, 
                "recy": 70+k*dy, 
                "r":    color_rgbs[k][0], 
                "g":    color_rgbs[k][1], 
                "b":    color_rgbs[k][2],
                "expr": mappings[k],
                "name": color_names[k]
            })
        text_file.write(tmpl3)
