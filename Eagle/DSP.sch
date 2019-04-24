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
<package name="TEENSY_3.2_SOCKET">
<pad name="GND_0" x="0" y="0" drill="1.016"/>
<pad name="0" x="0" y="-2.54" drill="1.016"/>
<pad name="1" x="0" y="-5.08" drill="1.016"/>
<pad name="2" x="0" y="-7.62" drill="1.016"/>
<pad name="3" x="0" y="-10.16" drill="1.016"/>
<pad name="4" x="0" y="-12.7" drill="1.016"/>
<pad name="5" x="0" y="-15.24" drill="1.016"/>
<pad name="6" x="0" y="-17.78" drill="1.016"/>
<pad name="7" x="0" y="-20.32" drill="1.016"/>
<pad name="8" x="0" y="-22.86" drill="1.016"/>
<pad name="9" x="0" y="-25.4" drill="1.016"/>
<pad name="10" x="0" y="-27.94" drill="1.016"/>
<pad name="12" x="0" y="-33.02" drill="1.016"/>
<pad name="3V3" x="5.08" y="-33.02" drill="1.016"/>
<pad name="GND_1" x="7.62" y="-33.02" drill="1.016"/>
<pad name="DAC" x="12.7" y="-33.02" drill="1.016"/>
<pad name="13" x="15.24" y="-33.02" drill="1.016"/>
<pad name="15" x="15.24" y="-27.94" drill="1.016"/>
<pad name="16" x="15.24" y="-25.4" drill="1.016"/>
<pad name="17" x="15.24" y="-22.86" drill="1.016"/>
<pad name="18" x="15.24" y="-20.32" drill="1.016"/>
<pad name="19" x="15.24" y="-17.78" drill="1.016"/>
<pad name="20" x="15.24" y="-15.24" drill="1.016"/>
<pad name="21" x="15.24" y="-12.7" drill="1.016"/>
<pad name="22" x="15.24" y="-10.16" drill="1.016"/>
<pad name="23" x="15.24" y="-7.62" drill="1.016"/>
<pad name="3V3_LIM" x="15.24" y="-5.08" drill="1.016"/>
<pad name="AGND" x="15.24" y="-2.54" drill="1.016"/>
<pad name="VIN" x="15.24" y="0" drill="1.016"/>
<text x="7.62" y="-20.32" size="1.27" layer="21" rot="R90">&gt;NAME</text>
<pad name="11" x="0" y="-30.48" drill="1.016"/>
<pad name="14" x="15.24" y="-30.48" drill="1.016"/>
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
</packages>
<symbols>
<symbol name="TEENSY_3.2_SOCKET">
<wire x1="-15.24" y1="22.86" x2="15.24" y2="22.86" width="0.254" layer="94"/>
<wire x1="15.24" y1="22.86" x2="15.24" y2="-22.86" width="0.254" layer="94"/>
<wire x1="15.24" y1="-22.86" x2="-15.24" y2="-22.86" width="0.254" layer="94"/>
<wire x1="-15.24" y1="-22.86" x2="-15.24" y2="22.86" width="0.254" layer="94"/>
<pin name="GND_0" x="-2.54" y="-27.94" length="middle" rot="R90"/>
<pin name="0" x="-20.32" y="15.24" length="middle"/>
<pin name="1" x="-20.32" y="12.7" length="middle"/>
<pin name="2" x="-20.32" y="10.16" length="middle"/>
<pin name="3" x="-20.32" y="7.62" length="middle"/>
<pin name="4" x="-20.32" y="5.08" length="middle"/>
<pin name="5" x="-20.32" y="2.54" length="middle"/>
<pin name="6" x="-20.32" y="0" length="middle"/>
<pin name="7" x="-20.32" y="-2.54" length="middle"/>
<pin name="8" x="-20.32" y="-5.08" length="middle"/>
<pin name="9" x="-20.32" y="-7.62" length="middle"/>
<pin name="10" x="-20.32" y="-10.16" length="middle"/>
<pin name="11" x="-20.32" y="-12.7" length="middle"/>
<pin name="12" x="-20.32" y="-15.24" length="middle"/>
<pin name="13" x="20.32" y="-15.24" length="middle" rot="R180"/>
<pin name="14" x="20.32" y="-12.7" length="middle" rot="R180"/>
<pin name="15" x="20.32" y="-10.16" length="middle" rot="R180"/>
<pin name="16" x="20.32" y="-7.62" length="middle" rot="R180"/>
<pin name="17" x="20.32" y="-5.08" length="middle" rot="R180"/>
<pin name="18" x="20.32" y="-2.54" length="middle" rot="R180"/>
<pin name="19" x="20.32" y="0" length="middle" rot="R180"/>
<pin name="20" x="20.32" y="2.54" length="middle" rot="R180"/>
<pin name="21" x="20.32" y="5.08" length="middle" rot="R180"/>
<pin name="22" x="20.32" y="7.62" length="middle" rot="R180"/>
<pin name="23" x="20.32" y="10.16" length="middle" rot="R180"/>
<pin name="3V3_LIM" x="5.08" y="27.94" length="middle" rot="R270"/>
<pin name="AGND" x="2.54" y="-27.94" length="middle" rot="R90"/>
<pin name="VIN" x="-5.08" y="27.94" length="middle" rot="R270"/>
<pin name="3V3" x="2.54" y="27.94" length="middle" rot="R270"/>
<pin name="GND_1" x="0" y="-27.94" length="middle" rot="R90"/>
<pin name="DAC" x="20.32" y="15.24" length="middle" rot="R180"/>
<text x="0" y="0" size="1.27" layer="94" align="bottom-center">&gt;NAME</text>
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
<deviceset name="TEENSY_3.2_SOCKET" prefix="U">
<gates>
<gate name="TEENSY_3.2" symbol="TEENSY_3.2_SOCKET" x="15.24" y="22.86"/>
</gates>
<devices>
<device name="" package="TEENSY_3.2_SOCKET">
<connects>
<connect gate="TEENSY_3.2" pin="0" pad="0"/>
<connect gate="TEENSY_3.2" pin="1" pad="1"/>
<connect gate="TEENSY_3.2" pin="10" pad="10"/>
<connect gate="TEENSY_3.2" pin="11" pad="11"/>
<connect gate="TEENSY_3.2" pin="12" pad="12"/>
<connect gate="TEENSY_3.2" pin="13" pad="13"/>
<connect gate="TEENSY_3.2" pin="14" pad="14"/>
<connect gate="TEENSY_3.2" pin="15" pad="15"/>
<connect gate="TEENSY_3.2" pin="16" pad="16"/>
<connect gate="TEENSY_3.2" pin="17" pad="17"/>
<connect gate="TEENSY_3.2" pin="18" pad="18"/>
<connect gate="TEENSY_3.2" pin="19" pad="19"/>
<connect gate="TEENSY_3.2" pin="2" pad="2"/>
<connect gate="TEENSY_3.2" pin="20" pad="20"/>
<connect gate="TEENSY_3.2" pin="21" pad="21"/>
<connect gate="TEENSY_3.2" pin="22" pad="22"/>
<connect gate="TEENSY_3.2" pin="23" pad="23"/>
<connect gate="TEENSY_3.2" pin="3" pad="3"/>
<connect gate="TEENSY_3.2" pin="3V3" pad="3V3"/>
<connect gate="TEENSY_3.2" pin="3V3_LIM" pad="3V3_LIM"/>
<connect gate="TEENSY_3.2" pin="4" pad="4"/>
<connect gate="TEENSY_3.2" pin="5" pad="5"/>
<connect gate="TEENSY_3.2" pin="6" pad="6"/>
<connect gate="TEENSY_3.2" pin="7" pad="7"/>
<connect gate="TEENSY_3.2" pin="8" pad="8"/>
<connect gate="TEENSY_3.2" pin="9" pad="9"/>
<connect gate="TEENSY_3.2" pin="AGND" pad="AGND"/>
<connect gate="TEENSY_3.2" pin="DAC" pad="DAC"/>
<connect gate="TEENSY_3.2" pin="GND_0" pad="GND_0"/>
<connect gate="TEENSY_3.2" pin="GND_1" pad="GND_1"/>
<connect gate="TEENSY_3.2" pin="VIN" pad="VIN"/>
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
<library name="supply2" urn="urn:adsk.eagle:library:372">
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
<symbol name="GND" urn="urn:adsk.eagle:symbol:26990/1" library_version="2">
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="0" y2="-1.27" width="0.254" layer="94"/>
<wire x1="0" y1="-1.27" x2="-1.27" y2="0" width="0.254" layer="94"/>
<text x="-1.905" y="-3.175" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" urn="urn:adsk.eagle:component:27037/1" prefix="SUPPLY" library_version="2">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="GND" symbol="GND" x="0" y="0"/>
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
<symbol name="+3V3" urn="urn:adsk.eagle:symbol:26950/1" library_version="1">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+3V3" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
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
<deviceset name="+3V3" urn="urn:adsk.eagle:component:26981/1" prefix="+3V3" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="+3V3" x="0" y="0"/>
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
<part name="TEENSY_3.2" library="kiss_z" deviceset="TEENSY_3.2_SOCKET" device=""/>
<part name="SUPPLY1" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="P+1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+5V" device=""/>
<part name="+3V1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="C1" library="kiss_z" deviceset="C0603" device="" value="0.1uF"/>
<part name="C2" library="kiss_z" deviceset="C0603" device="" value="0.1uF"/>
<part name="C3" library="kiss_z" deviceset="C0603" device=""/>
<part name="P+2" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+5V" device=""/>
<part name="+3V2" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="SUPPLY2" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY3" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="TEENSY_3.2" gate="TEENSY_3.2" x="63.5" y="45.72"/>
<instance part="SUPPLY1" gate="GND" x="68.58" y="5.08"/>
<instance part="P+1" gate="1" x="53.34" y="86.36"/>
<instance part="+3V1" gate="G$1" x="73.66" y="86.36"/>
<instance part="C1" gate="G$1" x="38.1" y="116.84"/>
<instance part="C2" gate="G$1" x="76.2" y="116.84"/>
<instance part="C3" gate="G$1" x="96.52" y="60.96" rot="R90"/>
<instance part="P+2" gate="1" x="33.02" y="127"/>
<instance part="+3V2" gate="G$1" x="71.12" y="127"/>
<instance part="SUPPLY2" gate="GND" x="38.1" y="104.14"/>
<instance part="SUPPLY3" gate="GND" x="76.2" y="104.14"/>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="TEENSY_3.2" gate="TEENSY_3.2" pin="GND_0"/>
<wire x1="60.96" y1="17.78" x2="60.96" y2="12.7" width="0.1524" layer="91"/>
<wire x1="60.96" y1="12.7" x2="63.5" y2="12.7" width="0.1524" layer="91"/>
<pinref part="TEENSY_3.2" gate="TEENSY_3.2" pin="GND_1"/>
<wire x1="63.5" y1="12.7" x2="63.5" y2="17.78" width="0.1524" layer="91"/>
<pinref part="TEENSY_3.2" gate="TEENSY_3.2" pin="AGND"/>
<wire x1="63.5" y1="12.7" x2="66.04" y2="12.7" width="0.1524" layer="91"/>
<wire x1="66.04" y1="12.7" x2="66.04" y2="17.78" width="0.1524" layer="91"/>
<junction x="63.5" y="12.7"/>
<pinref part="SUPPLY1" gate="GND" pin="GND"/>
<wire x1="66.04" y1="12.7" x2="68.58" y2="12.7" width="0.1524" layer="91"/>
<wire x1="68.58" y1="12.7" x2="68.58" y2="7.62" width="0.1524" layer="91"/>
<junction x="66.04" y="12.7"/>
</segment>
<segment>
<pinref part="C2" gate="G$1" pin="2"/>
<pinref part="SUPPLY3" gate="GND" pin="GND"/>
<wire x1="76.2" y1="106.68" x2="76.2" y2="111.76" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C1" gate="G$1" pin="2"/>
<pinref part="SUPPLY2" gate="GND" pin="GND"/>
<wire x1="38.1" y1="106.68" x2="38.1" y2="111.76" width="0.1524" layer="91"/>
</segment>
</net>
<net name="+5V" class="0">
<segment>
<pinref part="P+1" gate="1" pin="+5V"/>
<wire x1="53.34" y1="83.82" x2="53.34" y2="78.74" width="0.1524" layer="91"/>
<wire x1="53.34" y1="78.74" x2="58.42" y2="78.74" width="0.1524" layer="91"/>
<pinref part="TEENSY_3.2" gate="TEENSY_3.2" pin="VIN"/>
<wire x1="58.42" y1="78.74" x2="58.42" y2="73.66" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="P+2" gate="1" pin="+5V"/>
<wire x1="33.02" y1="124.46" x2="33.02" y2="121.92" width="0.1524" layer="91"/>
<wire x1="33.02" y1="121.92" x2="38.1" y2="121.92" width="0.1524" layer="91"/>
<pinref part="C1" gate="G$1" pin="1"/>
<wire x1="38.1" y1="121.92" x2="38.1" y2="119.38" width="0.1524" layer="91"/>
</segment>
</net>
<net name="+3V3" class="0">
<segment>
<pinref part="+3V1" gate="G$1" pin="+3V3"/>
<wire x1="73.66" y1="83.82" x2="73.66" y2="78.74" width="0.1524" layer="91"/>
<wire x1="73.66" y1="78.74" x2="68.58" y2="78.74" width="0.1524" layer="91"/>
<pinref part="TEENSY_3.2" gate="TEENSY_3.2" pin="3V3_LIM"/>
<wire x1="68.58" y1="78.74" x2="68.58" y2="73.66" width="0.1524" layer="91"/>
<pinref part="TEENSY_3.2" gate="TEENSY_3.2" pin="3V3"/>
<wire x1="68.58" y1="78.74" x2="66.04" y2="78.74" width="0.1524" layer="91"/>
<wire x1="66.04" y1="78.74" x2="66.04" y2="73.66" width="0.1524" layer="91"/>
<junction x="68.58" y="78.74"/>
</segment>
<segment>
<pinref part="+3V2" gate="G$1" pin="+3V3"/>
<wire x1="71.12" y1="124.46" x2="71.12" y2="121.92" width="0.1524" layer="91"/>
<wire x1="71.12" y1="121.92" x2="76.2" y2="121.92" width="0.1524" layer="91"/>
<pinref part="C2" gate="G$1" pin="1"/>
<wire x1="76.2" y1="121.92" x2="76.2" y2="119.38" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="TEENSY_3.2" gate="TEENSY_3.2" pin="DAC"/>
<pinref part="C3" gate="G$1" pin="1"/>
<wire x1="83.82" y1="60.96" x2="93.98" y2="60.96" width="0.1524" layer="91"/>
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
