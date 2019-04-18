<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="8.6.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="24" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="5" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="kiss_z">
<packages>
<package name="MTC16">
<smd name="1" x="-2.8448" y="2.275" dx="1.4224" dy="0.3556" layer="1"/>
<smd name="2" x="-2.8448" y="1.625" dx="1.4224" dy="0.3556" layer="1"/>
<smd name="3" x="-2.8448" y="0.975" dx="1.4224" dy="0.3556" layer="1"/>
<smd name="4" x="-2.8448" y="0.325" dx="1.4224" dy="0.3556" layer="1"/>
<smd name="5" x="-2.8448" y="-0.324996875" dx="1.4224" dy="0.3556" layer="1"/>
<smd name="6" x="-2.8448" y="-0.975" dx="1.4224" dy="0.3556" layer="1"/>
<smd name="7" x="-2.8448" y="-1.624996875" dx="1.4224" dy="0.3556" layer="1"/>
<smd name="8" x="-2.8448" y="-2.275" dx="1.4224" dy="0.3556" layer="1"/>
<smd name="9" x="2.8448" y="-2.275" dx="1.4224" dy="0.3556" layer="1" rot="R180"/>
<smd name="10" x="2.8448" y="-1.625" dx="1.4224" dy="0.3556" layer="1" rot="R180"/>
<smd name="11" x="2.8448" y="-0.975" dx="1.4224" dy="0.3556" layer="1" rot="R180"/>
<smd name="12" x="2.8448" y="-0.325" dx="1.4224" dy="0.3556" layer="1" rot="R180"/>
<smd name="13" x="2.8448" y="0.324996875" dx="1.4224" dy="0.3556" layer="1" rot="R180"/>
<smd name="14" x="2.8448" y="0.975" dx="1.4224" dy="0.3556" layer="1" rot="R180"/>
<smd name="15" x="2.8448" y="1.624996875" dx="1.4224" dy="0.3556" layer="1" rot="R180"/>
<smd name="16" x="2.8448" y="2.275" dx="1.4224" dy="0.3556" layer="1" rot="R180"/>
<wire x1="-2.2352" y1="2.1336" x2="-2.2606" y2="2.4384" width="0.1524" layer="51"/>
<wire x1="-2.2606" y1="2.4384" x2="-3.2004" y2="2.4384" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="2.4384" x2="-3.2004" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="2.1336" x2="-2.2352" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="1.4732" x2="-2.2606" y2="1.778" width="0.1524" layer="51"/>
<wire x1="-2.2606" y1="1.778" x2="-3.2004" y2="1.778" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="1.778" x2="-3.2004" y2="1.4732" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="1.4732" x2="-2.2352" y2="1.4732" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="0.8128" x2="-2.2352" y2="1.1176" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="1.1176" x2="-3.2004" y2="1.1176" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="1.1176" x2="-3.2004" y2="0.8128" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="0.8128" x2="-2.2352" y2="0.8128" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="0.1778" x2="-2.2352" y2="0.4826" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="0.4826" x2="-3.2004" y2="0.4826" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="0.4826" x2="-3.2004" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="0.1778" x2="-2.2352" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="-0.4826" x2="-2.2352" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="-0.1778" x2="-3.2004" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="-0.1778" x2="-3.2004" y2="-0.4826" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="-0.4826" x2="-2.2352" y2="-0.4826" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="-1.1176" x2="-2.2352" y2="-0.8128" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="-0.8128" x2="-3.2004" y2="-0.8128" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="-0.8128" x2="-3.2004" y2="-1.1176" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="-1.1176" x2="-2.2352" y2="-1.1176" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="-1.778" x2="-2.2352" y2="-1.4732" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="-1.4732" x2="-3.2004" y2="-1.4732" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="-1.4732" x2="-3.2004" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="-1.778" x2="-2.2352" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="-2.4384" x2="-2.2352" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="-2.1336" x2="-3.2004" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="-2.1336" x2="-3.2004" y2="-2.4384" width="0.1524" layer="51"/>
<wire x1="-3.2004" y1="-2.4384" x2="-2.2352" y2="-2.4384" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="-2.1336" x2="2.2606" y2="-2.4384" width="0.1524" layer="51"/>
<wire x1="2.2606" y1="-2.4384" x2="3.2004" y2="-2.4384" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="-2.4384" x2="3.2004" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="-2.1336" x2="2.2352" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="-1.4732" x2="2.2352" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="-1.778" x2="3.2004" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="-1.778" x2="3.2004" y2="-1.4732" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="-1.4732" x2="2.2352" y2="-1.4732" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="-0.8128" x2="2.2352" y2="-1.1176" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="-1.1176" x2="3.2004" y2="-1.1176" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="-1.1176" x2="3.2004" y2="-0.8128" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="-0.8128" x2="2.2352" y2="-0.8128" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="-0.1778" x2="2.2352" y2="-0.4826" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="-0.4826" x2="3.2004" y2="-0.4826" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="-0.4826" x2="3.2004" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="-0.1778" x2="2.2352" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="0.4826" x2="2.2352" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="0.1778" x2="3.2004" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="0.1778" x2="3.2004" y2="0.4826" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="0.4826" x2="2.2352" y2="0.4826" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="1.1176" x2="2.2352" y2="0.8128" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="0.8128" x2="3.2004" y2="0.8128" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="0.8128" x2="3.2004" y2="1.1176" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="1.1176" x2="2.2352" y2="1.1176" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="1.778" x2="2.2352" y2="1.4732" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="1.4732" x2="3.2004" y2="1.4732" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="1.4732" x2="3.2004" y2="1.778" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="1.778" x2="2.2352" y2="1.778" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="2.4384" x2="2.2352" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="2.1336" x2="3.2004" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="2.1336" x2="3.2004" y2="2.4384" width="0.1524" layer="51"/>
<wire x1="3.2004" y1="2.4384" x2="2.2352" y2="2.4384" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="-2.54" x2="2.2352" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="-2.54" x2="2.2352" y2="2.54" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="2.54" x2="-0.3048" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="2.54" x2="-2.2352" y2="2.54" width="0.1524" layer="51"/>
<wire x1="-2.2352" y1="2.54" x2="-2.2352" y2="-2.54" width="0.1524" layer="51"/>
<wire x1="-2.0574" y1="2.286" x2="-2.2098" y2="2.286" width="0" layer="51" curve="-180"/>
<wire x1="-2.2098" y1="2.286" x2="-2.0574" y2="2.286" width="0" layer="51" curve="-180"/>
<wire x1="0.3048" y1="2.5654" x2="-0.3048" y2="2.54" width="0.1524" layer="51" curve="-180"/>
<wire x1="-3.81" y1="-2.6416" x2="-3.81" y2="2.6416" width="0.1524" layer="39"/>
<wire x1="-3.81" y1="2.6416" x2="-2.4892" y2="2.6416" width="0.1524" layer="39"/>
<wire x1="3.81" y1="2.6416" x2="2.4892" y2="2.6416" width="0.1524" layer="39"/>
<wire x1="3.81" y1="2.6416" x2="3.81" y2="-2.6416" width="0.1524" layer="39"/>
<wire x1="3.81" y1="-2.6416" x2="2.4892" y2="-2.6416" width="0.1524" layer="39"/>
<wire x1="-3.81" y1="-2.6416" x2="-2.4892" y2="-2.6416" width="0.1524" layer="39"/>
<wire x1="-2.4892" y1="-2.794" x2="-2.4892" y2="-2.6416" width="0.1524" layer="39"/>
<wire x1="-2.4892" y1="2.6416" x2="-2.4892" y2="2.794" width="0.1524" layer="39"/>
<wire x1="-2.4892" y1="2.794" x2="2.4892" y2="2.794" width="0.1524" layer="39"/>
<wire x1="2.4892" y1="2.794" x2="2.4892" y2="2.6416" width="0.1524" layer="39"/>
<wire x1="2.4892" y1="-2.6416" x2="2.4892" y2="-2.794" width="0.1524" layer="39"/>
<wire x1="2.4892" y1="-2.794" x2="-2.4892" y2="-2.794" width="0.1524" layer="39"/>
<polygon width="0.1524" layer="39">
<vertex x="-3.81" y="-2.6508"/>
<vertex x="-3.81" y="2.6508"/>
<vertex x="-2.5019" y="2.6508"/>
<vertex x="-2.5019" y="2.8067"/>
<vertex x="2.5019" y="2.8067"/>
<vertex x="2.5019" y="2.6508"/>
<vertex x="3.81" y="2.6508"/>
<vertex x="3.81" y="-2.6508"/>
<vertex x="2.5019" y="-2.6508"/>
<vertex x="2.5019" y="-2.8067"/>
<vertex x="-2.5019" y="-2.8067"/>
<vertex x="-2.5019" y="-2.6508"/>
</polygon>
<wire x1="-1.8796" y1="-2.6924" x2="1.8796" y2="-2.6924" width="0.1524" layer="21"/>
<wire x1="1.8796" y1="2.6924" x2="-1.8796" y2="2.6924" width="0.1524" layer="21"/>
<wire x1="-3.9116" y1="2.286" x2="-4.064" y2="2.286" width="0.1524" layer="21" curve="-180"/>
<wire x1="-4.064" y1="2.286" x2="-3.9116" y2="2.286" width="0.1524" layer="21" curve="-180"/>
<wire x1="-0.254" y1="0" x2="0.254" y2="0" width="0.1524" layer="23"/>
<wire x1="0" y1="-0.254" x2="0" y2="0.254" width="0.1524" layer="23"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="25" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="ASTX-H11">
<wire x1="1" y1="1.25" x2="1" y2="-1.25" width="0.127" layer="51"/>
<wire x1="1" y1="-1.25" x2="-1" y2="-1.25" width="0.127" layer="51"/>
<wire x1="-1" y1="-1.25" x2="-1" y2="1.25" width="0.127" layer="51"/>
<wire x1="-1" y1="1.25" x2="1" y2="1.25" width="0.127" layer="51"/>
<text x="-1.839359375" y="2.65018125" size="0.8128" layer="25">&gt;NAME</text>
<circle x="-0.635" y="0.635" radius="0.2" width="0.127" layer="51"/>
<circle x="-1.835" y="0.805" radius="0.07" width="0.16" layer="21"/>
<wire x1="1.45" y1="1.5" x2="1.45" y2="-1.5" width="0.05" layer="39"/>
<wire x1="1.45" y1="-1.5" x2="-1.45" y2="-1.5" width="0.05" layer="39"/>
<wire x1="-1.45" y1="-1.5" x2="-1.45" y2="1.5" width="0.05" layer="39"/>
<wire x1="-1.45" y1="1.5" x2="1.45" y2="1.5" width="0.05" layer="39"/>
<smd name="4" x="0.775" y="0.825" dx="0.65" dy="0.85" layer="1" rot="R270"/>
<smd name="3" x="0.775" y="-0.825" dx="0.65" dy="0.85" layer="1" rot="R270"/>
<smd name="2" x="-0.775" y="-0.825" dx="0.65" dy="0.85" layer="1" rot="R270"/>
<smd name="1" x="-0.775" y="0.825" dx="0.65" dy="0.85" layer="1" rot="R270"/>
</package>
<package name="21-0036K">
<smd name="1" x="-2.1844" y="0.975" dx="1.3716" dy="0.4064" layer="1"/>
<smd name="2" x="-2.1844" y="0.325" dx="1.3716" dy="0.4064" layer="1"/>
<smd name="3" x="-2.1844" y="-0.325" dx="1.3716" dy="0.4064" layer="1"/>
<smd name="4" x="-2.1844" y="-0.975" dx="1.3716" dy="0.4064" layer="1"/>
<smd name="5" x="2.1844" y="-0.975" dx="1.3716" dy="0.4064" layer="1"/>
<smd name="6" x="2.1844" y="-0.325" dx="1.3716" dy="0.4064" layer="1"/>
<smd name="7" x="2.1844" y="0.325" dx="1.3716" dy="0.4064" layer="1"/>
<smd name="8" x="2.1844" y="0.975" dx="1.3716" dy="0.4064" layer="1"/>
<wire x1="-1.5494" y1="0.7874" x2="-1.5494" y2="1.143" width="0.1524" layer="51"/>
<wire x1="-1.5494" y1="1.143" x2="-2.5146" y2="1.143" width="0.1524" layer="51"/>
<wire x1="-2.5146" y1="1.143" x2="-2.5146" y2="0.7874" width="0.1524" layer="51"/>
<wire x1="-2.5146" y1="0.7874" x2="-1.5494" y2="0.7874" width="0.1524" layer="51"/>
<wire x1="-1.5494" y1="0.1524" x2="-1.5494" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-1.5494" y1="0.508" x2="-2.5146" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-2.5146" y1="0.508" x2="-2.5146" y2="0.1524" width="0.1524" layer="51"/>
<wire x1="-2.5146" y1="0.1524" x2="-1.5494" y2="0.1524" width="0.1524" layer="51"/>
<wire x1="-1.5494" y1="-0.508" x2="-1.5494" y2="-0.1524" width="0.1524" layer="51"/>
<wire x1="-1.5494" y1="-0.1524" x2="-2.5146" y2="-0.1524" width="0.1524" layer="51"/>
<wire x1="-2.5146" y1="-0.1524" x2="-2.5146" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-2.5146" y1="-0.508" x2="-1.5494" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-1.5494" y1="-1.143" x2="-1.5494" y2="-0.7874" width="0.1524" layer="51"/>
<wire x1="-1.5494" y1="-0.7874" x2="-2.5146" y2="-0.7874" width="0.1524" layer="51"/>
<wire x1="-2.5146" y1="-0.7874" x2="-2.5146" y2="-1.143" width="0.1524" layer="51"/>
<wire x1="-2.5146" y1="-1.143" x2="-1.5494" y2="-1.143" width="0.1524" layer="51"/>
<wire x1="1.5494" y1="-0.7874" x2="1.5494" y2="-1.143" width="0.1524" layer="51"/>
<wire x1="1.5494" y1="-1.143" x2="2.5146" y2="-1.143" width="0.1524" layer="51"/>
<wire x1="2.5146" y1="-1.143" x2="2.5146" y2="-0.7874" width="0.1524" layer="51"/>
<wire x1="2.5146" y1="-0.7874" x2="1.5494" y2="-0.7874" width="0.1524" layer="51"/>
<wire x1="1.5494" y1="-0.1524" x2="1.5494" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="1.5494" y1="-0.508" x2="2.5146" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="2.5146" y1="-0.508" x2="2.5146" y2="-0.1524" width="0.1524" layer="51"/>
<wire x1="2.5146" y1="-0.1524" x2="1.5494" y2="-0.1524" width="0.1524" layer="51"/>
<wire x1="1.5494" y1="0.508" x2="1.5494" y2="0.1524" width="0.1524" layer="51"/>
<wire x1="1.5494" y1="0.1524" x2="2.5146" y2="0.1524" width="0.1524" layer="51"/>
<wire x1="2.5146" y1="0.1524" x2="2.5146" y2="0.508" width="0.1524" layer="51"/>
<wire x1="2.5146" y1="0.508" x2="1.5494" y2="0.508" width="0.1524" layer="51"/>
<wire x1="1.5494" y1="1.143" x2="1.5494" y2="0.7874" width="0.1524" layer="51"/>
<wire x1="1.5494" y1="0.7874" x2="2.5146" y2="0.7874" width="0.1524" layer="51"/>
<wire x1="2.5146" y1="0.7874" x2="2.5146" y2="1.143" width="0.1524" layer="51"/>
<wire x1="2.5146" y1="1.143" x2="1.5494" y2="1.143" width="0.1524" layer="51"/>
<wire x1="-1.5494" y1="-1.5494" x2="1.5494" y2="-1.5494" width="0.1524" layer="51"/>
<wire x1="1.5494" y1="-1.5494" x2="1.5494" y2="1.5494" width="0.1524" layer="51"/>
<wire x1="1.5494" y1="1.5494" x2="0.3048" y2="1.5494" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="1.5494" x2="-0.3048" y2="1.5494" width="0.1524" layer="51"/>
<wire x1="-0.3048" y1="1.5494" x2="-1.5494" y2="1.5494" width="0.1524" layer="51"/>
<wire x1="-1.5494" y1="1.5494" x2="-1.5494" y2="-1.5494" width="0.1524" layer="51"/>
<wire x1="0.3048" y1="1.5494" x2="-0.3048" y2="1.5494" width="0.1524" layer="51" curve="-180"/>
<text x="-1.7526" y="0.2032" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<wire x1="-3.1242" y1="-1.4224" x2="-3.1242" y2="1.4224" width="0.1524" layer="39"/>
<wire x1="-3.1242" y1="1.4224" x2="-1.8034" y2="1.4224" width="0.1524" layer="39"/>
<wire x1="-1.8034" y1="1.4224" x2="-1.8034" y2="1.8034" width="0.1524" layer="39"/>
<wire x1="-1.8034" y1="1.8034" x2="1.8034" y2="1.8034" width="0.1524" layer="39"/>
<wire x1="1.8034" y1="1.8034" x2="1.8034" y2="1.4224" width="0.1524" layer="39"/>
<wire x1="3.1242" y1="1.4224" x2="1.8034" y2="1.4224" width="0.1524" layer="39"/>
<wire x1="3.1242" y1="1.4224" x2="3.1242" y2="-1.4224" width="0.1524" layer="39"/>
<wire x1="3.1242" y1="-1.4224" x2="1.8034" y2="-1.4224" width="0.1524" layer="39"/>
<wire x1="1.8034" y1="-1.4224" x2="1.8034" y2="-1.8034" width="0.1524" layer="39"/>
<wire x1="1.8034" y1="-1.8034" x2="-1.8034" y2="-1.8034" width="0.1524" layer="39"/>
<wire x1="-1.8034" y1="-1.8034" x2="-1.8034" y2="-1.4224" width="0.1524" layer="39"/>
<wire x1="-3.1242" y1="-1.4224" x2="-1.8034" y2="-1.4224" width="0.1524" layer="39"/>
<polygon width="0.1524" layer="39">
<vertex x="-3.1242" y="-1.4322"/>
<vertex x="-3.1242" y="1.4322"/>
<vertex x="-1.8034" y="1.4322"/>
<vertex x="-1.8034" y="1.8034"/>
<vertex x="1.8034" y="1.8034"/>
<vertex x="1.8034" y="1.4322"/>
<vertex x="3.1242" y="1.4322"/>
<vertex x="3.1242" y="-1.4322"/>
<vertex x="1.8034" y="-1.4322"/>
<vertex x="1.8034" y="-1.8034"/>
<vertex x="-1.8034" y="-1.8034"/>
<vertex x="-1.8034" y="-1.4322"/>
</polygon>
<wire x1="-1.6764" y1="-1.6764" x2="1.6764" y2="-1.6764" width="0.1524" layer="21"/>
<wire x1="1.6764" y1="-1.6764" x2="1.6764" y2="-1.4986" width="0.1524" layer="21"/>
<wire x1="1.6764" y1="1.6764" x2="-1.6764" y2="1.6764" width="0.1524" layer="21"/>
<wire x1="-1.6764" y1="1.6764" x2="-1.6764" y2="1.4986" width="0.1524" layer="21"/>
<wire x1="-1.6764" y1="-1.4986" x2="-1.6764" y2="-1.6764" width="0.1524" layer="21"/>
<wire x1="1.6764" y1="1.4986" x2="1.6764" y2="1.6764" width="0.1524" layer="21"/>
<text x="-3.0226" y="1.2446" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<wire x1="-0.254" y1="0" x2="0.254" y2="0" width="0.1524" layer="23"/>
<wire x1="0" y1="-0.254" x2="0" y2="0.254" width="0.1524" layer="23"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="25" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="0603">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.432" y1="-0.356" x2="0.432" y2="-0.356" width="0.1524" layer="51"/>
<wire x1="0.432" y1="0.356" x2="-0.432" y2="0.356" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1" dy="1.1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1" dy="1.1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4318" y1="-0.4318" x2="0.8382" y2="0.4318" layer="51"/>
<rectangle x1="-0.8382" y1="-0.4318" x2="-0.4318" y2="0.4318" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="TT1224">
<smd name="1" x="0" y="0" dx="1.524" dy="1.651" layer="1"/>
<smd name="2" x="2.54" y="0" dx="1.524" dy="1.651" layer="1"/>
<smd name="3" x="5.08" y="0" dx="1.524" dy="1.651" layer="1"/>
<smd name="4" x="5.08" y="5.969" dx="1.524" dy="1.651" layer="1" rot="R180"/>
<smd name="5" x="2.54" y="5.969" dx="1.524" dy="1.651" layer="1" rot="R180"/>
<smd name="6" x="0" y="5.969" dx="1.524" dy="1.651" layer="1" rot="R180"/>
<wire x1="-1.778" y1="-1.016" x2="-1.778" y2="-2.032" width="0.127" layer="21"/>
<wire x1="-2.286" y1="-1.524" x2="-1.27" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-1.524" y1="-1.27" x2="-2.032" y2="-1.778" width="0.127" layer="21"/>
<wire x1="-2.032" y1="-1.27" x2="-1.524" y2="-1.778" width="0.127" layer="21"/>
<text x="-1.27" y="0" size="0.8128" layer="27" ratio="6" rot="R90">&gt;NAME</text>
</package>
</packages>
<symbols>
<symbol name="LMX2326TM/NOPB">
<pin name="FLO" x="35.56" y="17.78" length="middle" direction="pas" rot="R180"/>
<pin name="CPO" x="35.56" y="20.32" length="middle" direction="pas" rot="R180"/>
<pin name="GND_2" x="15.24" y="-5.08" length="middle" direction="pas" rot="R90"/>
<pin name="GND_3" x="17.78" y="-5.08" length="middle" direction="pas" rot="R90"/>
<pin name="!FIN" x="35.56" y="10.16" length="middle" direction="pas" rot="R180"/>
<pin name="FIN" x="35.56" y="7.62" length="middle" direction="pas" rot="R180"/>
<pin name="VCC1" x="12.7" y="33.02" length="middle" direction="pas" rot="R270"/>
<pin name="OSCIN" x="-5.08" y="5.08" length="middle" direction="pas"/>
<pin name="GND" x="12.7" y="-5.08" length="middle" direction="pas" rot="R90"/>
<pin name="CE" x="-5.08" y="22.86" length="middle" direction="pas"/>
<pin name="CLOCK" x="-5.08" y="20.32" length="middle" direction="pas"/>
<pin name="DATA" x="-5.08" y="17.78" length="middle" direction="pas"/>
<pin name="LE" x="-5.08" y="15.24" length="middle" direction="pas"/>
<pin name="FO/LD" x="-5.08" y="10.16" length="middle" direction="pas"/>
<pin name="VCC2" x="15.24" y="33.02" length="middle" direction="pas" rot="R270"/>
<pin name="VP" x="17.78" y="33.02" length="middle" direction="pas" rot="R270"/>
<wire x1="0" y1="27.94" x2="0" y2="0" width="0.1524" layer="94"/>
<wire x1="0" y1="0" x2="30.48" y2="0" width="0.1524" layer="94"/>
<wire x1="30.48" y1="0" x2="30.48" y2="27.94" width="0.1524" layer="94"/>
<wire x1="30.48" y1="27.94" x2="0" y2="27.94" width="0.1524" layer="94"/>
<text x="25.7556" y="31.9786" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="25.1206" y="29.4386" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
<symbol name="ASTX-H11">
<wire x1="-10.16" y1="10.16" x2="10.16" y2="10.16" width="0.254" layer="94"/>
<wire x1="10.16" y1="10.16" x2="10.16" y2="-10.16" width="0.254" layer="94"/>
<wire x1="10.16" y1="-10.16" x2="-10.16" y2="-10.16" width="0.254" layer="94"/>
<wire x1="-10.16" y1="-10.16" x2="-10.16" y2="10.16" width="0.254" layer="94"/>
<pin name="EN" x="-15.24" y="0" length="middle" direction="in"/>
<pin name="GND" x="0" y="-15.24" length="middle" direction="pwr" rot="R90"/>
<pin name="OUT" x="15.24" y="0" length="middle" direction="out" rot="R180"/>
<pin name="VDD" x="0" y="15.24" length="middle" direction="pwr" rot="R270"/>
<text x="2.54" y="15.24" size="1.27" layer="95" ratio="6">&gt;NAME</text>
<text x="2.54" y="12.7" size="1.27" layer="96" ratio="6">&gt;VALUE</text>
</symbol>
<symbol name="MAX2752EUA+">
<pin name="BYP" x="-5.08" y="15.24" length="middle" direction="pas"/>
<pin name="TUNE" x="-5.08" y="5.08" length="middle" direction="pas"/>
<pin name="GND" x="10.16" y="-5.08" length="middle" direction="pwr" rot="R90"/>
<pin name="!SHDN" x="-5.08" y="12.7" length="middle" direction="pas"/>
<pin name="VCC1" x="10.16" y="25.4" length="middle" direction="pwr" rot="R270"/>
<pin name="VCC2" x="12.7" y="25.4" length="middle" direction="pwr" rot="R270"/>
<pin name="OUT" x="27.94" y="10.16" length="middle" direction="out" rot="R180"/>
<pin name="GND_2" x="12.7" y="-5.08" length="middle" direction="pwr" rot="R90"/>
<wire x1="0" y1="20.32" x2="0" y2="0" width="0.1524" layer="94"/>
<wire x1="0" y1="0" x2="22.86" y2="0" width="0.1524" layer="94"/>
<wire x1="22.86" y1="0" x2="22.86" y2="20.32" width="0.1524" layer="94"/>
<wire x1="22.86" y1="20.32" x2="0" y2="20.32" width="0.1524" layer="94"/>
<text x="18.1356" y="24.3586" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="17.5006" y="21.8186" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
<symbol name="R">
<wire x1="-2.54" y1="0" x2="-2.159" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-2.159" y1="1.016" x2="-1.524" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-1.524" y1="-1.016" x2="-0.889" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-0.889" y1="1.016" x2="-0.254" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-0.254" y1="-1.016" x2="0.381" y2="1.016" width="0.2032" layer="94"/>
<wire x1="0.381" y1="1.016" x2="1.016" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="1.016" y1="-1.016" x2="1.651" y2="1.016" width="0.2032" layer="94"/>
<wire x1="1.651" y1="1.016" x2="2.286" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="0" width="0.2032" layer="94"/>
<text x="-3.81" y="1.4986" size="1.778" layer="95">&gt;NAME</text>
<text x="-3.81" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
</symbol>
<symbol name="RMK-3-93+">
<wire x1="0" y1="17.78" x2="27.94" y2="17.78" width="0.254" layer="94"/>
<wire x1="27.94" y1="17.78" x2="27.94" y2="0" width="0.254" layer="94"/>
<wire x1="27.94" y1="0" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="17.78" width="0.254" layer="94"/>
<pin name="INPUT" x="-5.08" y="12.7" length="middle" direction="in"/>
<pin name="GND" x="10.16" y="-5.08" length="middle" direction="pwr" rot="R90"/>
<pin name="GND_2" x="12.7" y="-5.08" length="middle" direction="pwr" rot="R90"/>
<pin name="OUTPUT" x="33.02" y="12.7" length="middle" direction="out" rot="R180"/>
<pin name="GND_3" x="15.24" y="-5.08" length="middle" direction="pwr" rot="R90"/>
<pin name="GND_4" x="17.78" y="-5.08" length="middle" direction="pwr" rot="R90"/>
<text x="19.05" y="21.59" size="1.778" layer="94">&gt;NAME</text>
<text x="19.05" y="19.05" size="1.778" layer="94">&gt;VALUE</text>
</symbol>
<symbol name="C">
<wire x1="0" y1="-2.54" x2="0" y2="-2.032" width="0.1524" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="-0.508" width="0.1524" layer="94"/>
<text x="1.524" y="0.381" size="1.778" layer="95">&gt;NAME</text>
<text x="1.524" y="-4.699" size="1.778" layer="96">&gt;VALUE</text>
<rectangle x1="-2.032" y1="-1.016" x2="2.032" y2="-0.508" layer="94"/>
<rectangle x1="-2.032" y1="-2.032" x2="2.032" y2="-1.524" layer="94"/>
<pin name="1" x="0" y="2.54" visible="off" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="2" x="0" y="-5.08" visible="off" length="short" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="LMX2326" prefix="U">
<gates>
<gate name="A" symbol="LMX2326TM/NOPB" x="-17.78" y="22.86"/>
</gates>
<devices>
<device name="" package="MTC16">
<connects>
<connect gate="A" pin="!FIN" pad="5"/>
<connect gate="A" pin="CE" pad="10"/>
<connect gate="A" pin="CLOCK" pad="11"/>
<connect gate="A" pin="CPO" pad="2"/>
<connect gate="A" pin="DATA" pad="12"/>
<connect gate="A" pin="FIN" pad="6"/>
<connect gate="A" pin="FLO" pad="1"/>
<connect gate="A" pin="FO/LD" pad="14"/>
<connect gate="A" pin="GND" pad="3"/>
<connect gate="A" pin="GND_2" pad="4"/>
<connect gate="A" pin="GND_3" pad="9"/>
<connect gate="A" pin="LE" pad="13"/>
<connect gate="A" pin="OSCIN" pad="8"/>
<connect gate="A" pin="VCC1" pad="7"/>
<connect gate="A" pin="VCC2" pad="15"/>
<connect gate="A" pin="VP" pad="16"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="ASTX-H11" prefix="XO">
<gates>
<gate name="A" symbol="ASTX-H11" x="10.16" y="10.16"/>
</gates>
<devices>
<device name="" package="ASTX-H11">
<connects>
<connect gate="A" pin="EN" pad="1"/>
<connect gate="A" pin="GND" pad="2"/>
<connect gate="A" pin="OUT" pad="3"/>
<connect gate="A" pin="VDD" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MAX2752" prefix="U">
<gates>
<gate name="A" symbol="MAX2752EUA+" x="-20.32" y="12.7"/>
</gates>
<devices>
<device name="" package="21-0036K">
<connects>
<connect gate="A" pin="!SHDN" pad="4"/>
<connect gate="A" pin="BYP" pad="1"/>
<connect gate="A" pin="GND" pad="3"/>
<connect gate="A" pin="GND_2" pad="8"/>
<connect gate="A" pin="OUT" pad="7"/>
<connect gate="A" pin="TUNE" pad="2"/>
<connect gate="A" pin="VCC1" pad="5"/>
<connect gate="A" pin="VCC2" pad="6"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="R0603" prefix="R" uservalue="yes">
<gates>
<gate name="G$1" symbol="R" x="5.08" y="0"/>
</gates>
<devices>
<device name="" package="0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="RMK-3-93+" prefix="U">
<gates>
<gate name="A" symbol="RMK-3-93+" x="0" y="0"/>
</gates>
<devices>
<device name="" package="TT1224">
<connects>
<connect gate="A" pin="GND" pad="2"/>
<connect gate="A" pin="GND_2" pad="3"/>
<connect gate="A" pin="GND_3" pad="5"/>
<connect gate="A" pin="GND_4" pad="6"/>
<connect gate="A" pin="INPUT" pad="1"/>
<connect gate="A" pin="OUTPUT" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="C0603" prefix="C" uservalue="yes">
<gates>
<gate name="G$1" symbol="C" x="0" y="5.08"/>
</gates>
<devices>
<device name="" package="0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply1" urn="urn:adsk.eagle:library:371">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
 GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
 Please keep in mind, that these devices are necessary for the
 automatic wiring of the supply signals.&lt;p&gt;
 The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
 In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
 &lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="+5V" urn="urn:adsk.eagle:symbol:26929/1" library_version="1">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+5V" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="GND" urn="urn:adsk.eagle:symbol:26925/1" library_version="1">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="+5V" urn="urn:adsk.eagle:component:26963/1" prefix="P+" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="+5V" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="GND" urn="urn:adsk.eagle:component:26954/1" prefix="GND" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U1" library="kiss_z" deviceset="LMX2326" device=""/>
<part name="XO1" library="kiss_z" deviceset="ASTX-H11" device=""/>
<part name="U2" library="kiss_z" deviceset="MAX2752" device=""/>
<part name="R1" library="kiss_z" deviceset="R0603" device=""/>
<part name="U3" library="kiss_z" deviceset="RMK-3-93+" device=""/>
<part name="C1" library="kiss_z" deviceset="C0603" device=""/>
<part name="C2" library="kiss_z" deviceset="C0603" device=""/>
<part name="P+1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+5V" device=""/>
<part name="GND1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND2" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND3" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND4" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U1" gate="A" x="111.76" y="78.74"/>
<instance part="XO1" gate="A" x="33.02" y="45.72"/>
<instance part="U2" gate="A" x="124.46" y="25.4"/>
<instance part="R1" gate="G$1" x="76.2" y="78.74"/>
<instance part="U3" gate="A" x="193.04" y="48.26"/>
<instance part="C1" gate="G$1" x="177.8" y="60.96" rot="R90"/>
<instance part="C2" gate="G$1" x="233.68" y="60.96" rot="R90"/>
<instance part="P+1" gate="1" x="119.38" y="116.84"/>
<instance part="GND1" gate="1" x="213.36" y="35.56"/>
<instance part="GND2" gate="1" x="139.7" y="12.7"/>
<instance part="GND3" gate="1" x="132.08" y="66.04"/>
<instance part="GND4" gate="1" x="33.02" y="25.4"/>
</instances>
<busses>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<pinref part="U3" gate="A" pin="INPUT"/>
<pinref part="C1" gate="G$1" pin="2"/>
<wire x1="182.88" y1="60.96" x2="187.96" y2="60.96" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="U3" gate="A" pin="OUTPUT"/>
<pinref part="C2" gate="G$1" pin="1"/>
<wire x1="231.14" y1="60.96" x2="226.06" y2="60.96" width="0.1524" layer="91"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="U3" gate="A" pin="GND"/>
<wire x1="203.2" y1="43.18" x2="203.2" y2="40.64" width="0.1524" layer="91"/>
<wire x1="203.2" y1="40.64" x2="205.74" y2="40.64" width="0.1524" layer="91"/>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="205.74" y1="40.64" x2="208.28" y2="40.64" width="0.1524" layer="91"/>
<wire x1="208.28" y1="40.64" x2="210.82" y2="40.64" width="0.1524" layer="91"/>
<wire x1="210.82" y1="40.64" x2="213.36" y2="40.64" width="0.1524" layer="91"/>
<wire x1="213.36" y1="40.64" x2="213.36" y2="38.1" width="0.1524" layer="91"/>
<pinref part="U3" gate="A" pin="GND_2"/>
<wire x1="205.74" y1="43.18" x2="205.74" y2="40.64" width="0.1524" layer="91"/>
<junction x="205.74" y="40.64"/>
<pinref part="U3" gate="A" pin="GND_3"/>
<wire x1="208.28" y1="43.18" x2="208.28" y2="40.64" width="0.1524" layer="91"/>
<junction x="208.28" y="40.64"/>
<pinref part="U3" gate="A" pin="GND_4"/>
<wire x1="210.82" y1="43.18" x2="210.82" y2="40.64" width="0.1524" layer="91"/>
<junction x="210.82" y="40.64"/>
</segment>
<segment>
<pinref part="XO1" gate="A" pin="GND"/>
<pinref part="GND4" gate="1" pin="GND"/>
<wire x1="33.02" y1="27.94" x2="33.02" y2="30.48" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U2" gate="A" pin="GND"/>
<wire x1="134.62" y1="20.32" x2="134.62" y2="17.78" width="0.1524" layer="91"/>
<wire x1="134.62" y1="17.78" x2="137.16" y2="17.78" width="0.1524" layer="91"/>
<pinref part="U2" gate="A" pin="GND_2"/>
<wire x1="137.16" y1="17.78" x2="137.16" y2="20.32" width="0.1524" layer="91"/>
<pinref part="GND2" gate="1" pin="GND"/>
<wire x1="137.16" y1="17.78" x2="139.7" y2="17.78" width="0.1524" layer="91"/>
<wire x1="139.7" y1="17.78" x2="139.7" y2="15.24" width="0.1524" layer="91"/>
<junction x="137.16" y="17.78"/>
</segment>
<segment>
<pinref part="U1" gate="A" pin="GND"/>
<wire x1="124.46" y1="73.66" x2="124.46" y2="71.12" width="0.1524" layer="91"/>
<wire x1="124.46" y1="71.12" x2="127" y2="71.12" width="0.1524" layer="91"/>
<pinref part="GND3" gate="1" pin="GND"/>
<wire x1="127" y1="71.12" x2="129.54" y2="71.12" width="0.1524" layer="91"/>
<wire x1="129.54" y1="71.12" x2="132.08" y2="71.12" width="0.1524" layer="91"/>
<wire x1="132.08" y1="71.12" x2="132.08" y2="68.58" width="0.1524" layer="91"/>
<pinref part="U1" gate="A" pin="GND_2"/>
<wire x1="127" y1="73.66" x2="127" y2="71.12" width="0.1524" layer="91"/>
<junction x="127" y="71.12"/>
<pinref part="U1" gate="A" pin="GND_3"/>
<wire x1="129.54" y1="73.66" x2="129.54" y2="71.12" width="0.1524" layer="91"/>
<junction x="129.54" y="71.12"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
