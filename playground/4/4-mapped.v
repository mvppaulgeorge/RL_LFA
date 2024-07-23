// Benchmark "adder" written by ABC on Wed Jul 17 13:57:59 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n334, new_n337, new_n338, new_n340, new_n342;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1d32x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  and002aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o(new_n102));
  inv040aa1d32x5               g007(.a(\a[3] ), .o1(new_n103));
  inv040aa1d28x5               g008(.a(\b[2] ), .o1(new_n104));
  nand02aa1d08x5               g009(.a(new_n104), .b(new_n103), .o1(new_n105));
  nanp02aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand02aa1n02x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  nor042aa1n03x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nand02aa1n08x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  nand42aa1n06x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  aoi012aa1n12x5               g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\a[4] ), .o1(new_n112));
  aboi22aa1d24x5               g017(.a(\b[3] ), .b(new_n112), .c(new_n103), .d(new_n104), .out0(new_n113));
  oaoi13aa1n12x5               g018(.a(new_n102), .b(new_n113), .c(new_n111), .d(new_n107), .o1(new_n114));
  nor022aa1n04x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nand22aa1n06x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nor022aa1n08x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nand42aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nona23aa1n09x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  tech160nm_fixnrc02aa1n04x5   g024(.a(\b[5] ), .b(\a[6] ), .out0(new_n120));
  xnrc02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .out0(new_n121));
  nor043aa1n02x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  norp02aa1n02x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  aoi112aa1n03x5               g028(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n124));
  nor002aa1n02x5               g029(.a(new_n124), .b(new_n123), .o1(new_n125));
  tech160nm_fiao0012aa1n02p5x5 g030(.a(new_n115), .b(new_n117), .c(new_n116), .o(new_n126));
  oabi12aa1n03x5               g031(.a(new_n126), .b(new_n119), .c(new_n125), .out0(new_n127));
  nand42aa1n10x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n127), .c(new_n114), .d(new_n122), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n101), .out0(\s[10] ));
  tech160nm_fioai012aa1n03p5x5 g035(.a(new_n113), .b(new_n111), .c(new_n107), .o1(new_n131));
  nor042aa1n02x5               g036(.a(new_n121), .b(new_n120), .o1(new_n132));
  nona23aa1n09x5               g037(.a(new_n131), .b(new_n132), .c(new_n119), .d(new_n102), .out0(new_n133));
  oab012aa1n06x5               g038(.a(new_n126), .b(new_n119), .c(new_n125), .out0(new_n134));
  nand22aa1n03x5               g039(.a(new_n133), .b(new_n134), .o1(new_n135));
  nano23aa1d15x5               g040(.a(new_n97), .b(new_n100), .c(new_n128), .d(new_n98), .out0(new_n136));
  aoi012aa1d24x5               g041(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n137));
  inv000aa1d42x5               g042(.a(new_n137), .o1(new_n138));
  nor002aa1d32x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand02aa1n10x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  aoai13aa1n06x5               g046(.a(new_n141), .b(new_n138), .c(new_n135), .d(new_n136), .o1(new_n142));
  aoi112aa1n02x5               g047(.a(new_n141), .b(new_n138), .c(new_n135), .d(new_n136), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n142), .b(new_n143), .out0(\s[11] ));
  inv000aa1d42x5               g049(.a(new_n139), .o1(new_n145));
  norp02aa1n24x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand02aa1n12x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  xnbna2aa1n03x5               g053(.a(new_n148), .b(new_n142), .c(new_n145), .out0(\s[12] ));
  nona23aa1d18x5               g054(.a(new_n147), .b(new_n140), .c(new_n139), .d(new_n146), .out0(new_n150));
  aoi012aa1n12x5               g055(.a(new_n146), .b(new_n139), .c(new_n147), .o1(new_n151));
  oai012aa1d24x5               g056(.a(new_n151), .b(new_n150), .c(new_n137), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nano23aa1d12x5               g058(.a(new_n139), .b(new_n146), .c(new_n147), .d(new_n140), .out0(new_n154));
  nand22aa1n12x5               g059(.a(new_n154), .b(new_n136), .o1(new_n155));
  aoai13aa1n06x5               g060(.a(new_n153), .b(new_n155), .c(new_n133), .d(new_n134), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n04x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1d42x5               g066(.a(new_n155), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n127), .c(new_n114), .d(new_n122), .o1(new_n163));
  nor042aa1n06x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand22aa1n09x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  aoi012aa1d18x5               g070(.a(new_n164), .b(new_n158), .c(new_n165), .o1(new_n166));
  nona23aa1n03x5               g071(.a(new_n165), .b(new_n159), .c(new_n158), .d(new_n164), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n166), .b(new_n167), .c(new_n163), .d(new_n153), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nand02aa1d06x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanb02aa1n06x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  inv020aa1n02x5               g077(.a(new_n172), .o1(new_n173));
  nor002aa1n03x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand02aa1d04x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  inv000aa1n02x5               g081(.a(new_n176), .o1(new_n177));
  aoi112aa1n02x5               g082(.a(new_n177), .b(new_n170), .c(new_n168), .d(new_n173), .o1(new_n178));
  inv000aa1d42x5               g083(.a(new_n170), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n166), .o1(new_n180));
  nano23aa1n06x5               g085(.a(new_n158), .b(new_n164), .c(new_n165), .d(new_n159), .out0(new_n181));
  aoai13aa1n02x5               g086(.a(new_n173), .b(new_n180), .c(new_n156), .d(new_n181), .o1(new_n182));
  tech160nm_fiaoi012aa1n02p5x5 g087(.a(new_n176), .b(new_n182), .c(new_n179), .o1(new_n183));
  norp02aa1n02x5               g088(.a(new_n183), .b(new_n178), .o1(\s[16] ));
  nona23aa1n09x5               g089(.a(new_n175), .b(new_n171), .c(new_n170), .d(new_n174), .out0(new_n185));
  nona23aa1n09x5               g090(.a(new_n181), .b(new_n136), .c(new_n185), .d(new_n150), .out0(new_n186));
  nor042aa1n03x5               g091(.a(new_n185), .b(new_n167), .o1(new_n187));
  aoi012aa1n02x5               g092(.a(new_n174), .b(new_n170), .c(new_n175), .o1(new_n188));
  oai012aa1n09x5               g093(.a(new_n188), .b(new_n185), .c(new_n166), .o1(new_n189));
  aoi012aa1d24x5               g094(.a(new_n189), .b(new_n152), .c(new_n187), .o1(new_n190));
  aoai13aa1n12x5               g095(.a(new_n190), .b(new_n186), .c(new_n133), .d(new_n134), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g097(.a(\a[18] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[16] ), .o1(new_n195));
  oaoi03aa1n03x5               g100(.a(new_n194), .b(new_n195), .c(new_n191), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n193), .out0(\s[18] ));
  nano32aa1n03x7               g102(.a(new_n155), .b(new_n177), .c(new_n173), .d(new_n181), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n127), .c(new_n114), .d(new_n122), .o1(new_n199));
  xroi22aa1d06x4               g104(.a(new_n194), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n200));
  inv000aa1n02x5               g105(.a(new_n200), .o1(new_n201));
  nor002aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  aoi112aa1n09x5               g107(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n203));
  nor002aa1n02x5               g108(.a(new_n203), .b(new_n202), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n204), .b(new_n201), .c(new_n199), .d(new_n190), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n12x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  xorc02aa1n03x5               g113(.a(\a[19] ), .b(\b[18] ), .out0(new_n209));
  nor042aa1n06x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nand42aa1n20x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  norb02aa1n02x7               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  aoi112aa1n03x4               g117(.a(new_n208), .b(new_n212), .c(new_n205), .d(new_n209), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n208), .o1(new_n214));
  inv020aa1n04x5               g119(.a(new_n204), .o1(new_n215));
  aoai13aa1n03x5               g120(.a(new_n209), .b(new_n215), .c(new_n191), .d(new_n200), .o1(new_n216));
  nanb02aa1n02x5               g121(.a(new_n210), .b(new_n211), .out0(new_n217));
  aoi012aa1n03x5               g122(.a(new_n217), .b(new_n216), .c(new_n214), .o1(new_n218));
  norp02aa1n03x5               g123(.a(new_n218), .b(new_n213), .o1(\s[20] ));
  xnrc02aa1n02x5               g124(.a(\b[18] ), .b(\a[19] ), .out0(new_n220));
  nor042aa1n06x5               g125(.a(new_n220), .b(new_n217), .o1(new_n221));
  nand02aa1d04x5               g126(.a(new_n200), .b(new_n221), .o1(new_n222));
  tech160nm_fiaoi012aa1n04x5   g127(.a(new_n210), .b(new_n208), .c(new_n211), .o1(new_n223));
  inv020aa1n03x5               g128(.a(new_n223), .o1(new_n224));
  aoi012aa1d18x5               g129(.a(new_n224), .b(new_n221), .c(new_n215), .o1(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n222), .c(new_n199), .d(new_n190), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  nand42aa1n16x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n229), .b(new_n228), .out0(new_n230));
  nor042aa1n06x5               g135(.a(\b[21] ), .b(\a[22] ), .o1(new_n231));
  nanp02aa1n12x5               g136(.a(\b[21] ), .b(\a[22] ), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n232), .b(new_n231), .out0(new_n233));
  aoi112aa1n03x4               g138(.a(new_n228), .b(new_n233), .c(new_n226), .d(new_n230), .o1(new_n234));
  inv040aa1n04x5               g139(.a(new_n228), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n222), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n225), .o1(new_n237));
  aoai13aa1n03x5               g142(.a(new_n230), .b(new_n237), .c(new_n191), .d(new_n236), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n233), .o1(new_n239));
  aoi012aa1n03x5               g144(.a(new_n239), .b(new_n238), .c(new_n235), .o1(new_n240));
  nor042aa1n03x5               g145(.a(new_n240), .b(new_n234), .o1(\s[22] ));
  nona23aa1n08x5               g146(.a(new_n232), .b(new_n229), .c(new_n228), .d(new_n231), .out0(new_n242));
  oaoi03aa1n12x5               g147(.a(\a[22] ), .b(\b[21] ), .c(new_n235), .o1(new_n243));
  oab012aa1n06x5               g148(.a(new_n243), .b(new_n225), .c(new_n242), .out0(new_n244));
  nano23aa1d12x5               g149(.a(new_n228), .b(new_n231), .c(new_n232), .d(new_n229), .out0(new_n245));
  nand23aa1d12x5               g150(.a(new_n200), .b(new_n221), .c(new_n245), .o1(new_n246));
  aoai13aa1n04x5               g151(.a(new_n244), .b(new_n246), .c(new_n199), .d(new_n190), .o1(new_n247));
  nor002aa1d24x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  nand22aa1n09x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n248), .out0(new_n250));
  inv040aa1n03x5               g155(.a(new_n244), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n246), .o1(new_n252));
  aoi112aa1n02x5               g157(.a(new_n251), .b(new_n250), .c(new_n191), .d(new_n252), .o1(new_n253));
  aoi012aa1n02x5               g158(.a(new_n253), .b(new_n247), .c(new_n250), .o1(\s[23] ));
  nor042aa1n04x5               g159(.a(\b[23] ), .b(\a[24] ), .o1(new_n255));
  nand02aa1d08x5               g160(.a(\b[23] ), .b(\a[24] ), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(new_n257));
  aoi112aa1n03x4               g162(.a(new_n248), .b(new_n257), .c(new_n247), .d(new_n250), .o1(new_n258));
  inv040aa1n02x5               g163(.a(new_n248), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n250), .b(new_n251), .c(new_n191), .d(new_n252), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n257), .o1(new_n261));
  aoi012aa1n03x5               g166(.a(new_n261), .b(new_n260), .c(new_n259), .o1(new_n262));
  norp02aa1n03x5               g167(.a(new_n262), .b(new_n258), .o1(\s[24] ));
  nano23aa1n09x5               g168(.a(new_n248), .b(new_n255), .c(new_n256), .d(new_n249), .out0(new_n264));
  nano32aa1n02x4               g169(.a(new_n201), .b(new_n264), .c(new_n221), .d(new_n245), .out0(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  oai112aa1n03x5               g171(.a(new_n209), .b(new_n212), .c(new_n203), .d(new_n202), .o1(new_n267));
  nand02aa1d04x5               g172(.a(new_n264), .b(new_n245), .o1(new_n268));
  oaoi03aa1n02x5               g173(.a(\a[24] ), .b(\b[23] ), .c(new_n259), .o1(new_n269));
  aoi012aa1n09x5               g174(.a(new_n269), .b(new_n264), .c(new_n243), .o1(new_n270));
  aoai13aa1n12x5               g175(.a(new_n270), .b(new_n268), .c(new_n267), .d(new_n223), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n266), .c(new_n199), .d(new_n190), .o1(new_n273));
  xorb03aa1n02x5               g178(.a(new_n273), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g179(.a(\b[24] ), .b(\a[25] ), .o1(new_n275));
  xorc02aa1n02x5               g180(.a(\a[25] ), .b(\b[24] ), .out0(new_n276));
  xorc02aa1n12x5               g181(.a(\a[26] ), .b(\b[25] ), .out0(new_n277));
  aoi112aa1n03x4               g182(.a(new_n275), .b(new_n277), .c(new_n273), .d(new_n276), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n275), .o1(new_n279));
  aoai13aa1n03x5               g184(.a(new_n276), .b(new_n271), .c(new_n191), .d(new_n265), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n277), .o1(new_n281));
  aoi012aa1n03x5               g186(.a(new_n281), .b(new_n280), .c(new_n279), .o1(new_n282));
  nor002aa1n02x5               g187(.a(new_n282), .b(new_n278), .o1(\s[26] ));
  nanp02aa1n02x5               g188(.a(new_n277), .b(new_n276), .o1(new_n284));
  inv000aa1n02x5               g189(.a(new_n284), .o1(new_n285));
  nano22aa1d15x5               g190(.a(new_n246), .b(new_n285), .c(new_n264), .out0(new_n286));
  inv020aa1n03x5               g191(.a(new_n286), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[26] ), .b(\b[25] ), .c(new_n279), .carry(new_n288));
  aobi12aa1n06x5               g193(.a(new_n288), .b(new_n271), .c(new_n285), .out0(new_n289));
  aoai13aa1n06x5               g194(.a(new_n289), .b(new_n287), .c(new_n199), .d(new_n190), .o1(new_n290));
  xorb03aa1n03x5               g195(.a(new_n290), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n04x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  inv000aa1n03x5               g197(.a(new_n292), .o1(new_n293));
  xorc02aa1n12x5               g198(.a(\a[28] ), .b(\b[27] ), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  nona23aa1n02x4               g200(.a(new_n256), .b(new_n249), .c(new_n248), .d(new_n255), .out0(new_n296));
  nor042aa1n03x5               g201(.a(new_n296), .b(new_n242), .o1(new_n297));
  aoai13aa1n02x5               g202(.a(new_n297), .b(new_n224), .c(new_n221), .d(new_n215), .o1(new_n298));
  aoai13aa1n04x5               g203(.a(new_n288), .b(new_n284), .c(new_n298), .d(new_n270), .o1(new_n299));
  nanp02aa1n02x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n299), .c(new_n191), .d(new_n286), .o1(new_n301));
  aoi012aa1n03x5               g206(.a(new_n295), .b(new_n301), .c(new_n293), .o1(new_n302));
  aoi112aa1n03x4               g207(.a(new_n292), .b(new_n294), .c(new_n290), .d(new_n300), .o1(new_n303));
  norp02aa1n03x5               g208(.a(new_n302), .b(new_n303), .o1(\s[28] ));
  nano22aa1n02x4               g209(.a(new_n295), .b(new_n293), .c(new_n300), .out0(new_n305));
  aoai13aa1n02x7               g210(.a(new_n305), .b(new_n299), .c(new_n191), .d(new_n286), .o1(new_n306));
  tech160nm_fioaoi03aa1n03p5x5 g211(.a(\a[28] ), .b(\b[27] ), .c(new_n293), .o1(new_n307));
  inv000aa1n03x5               g212(.a(new_n307), .o1(new_n308));
  tech160nm_fixorc02aa1n03p5x5 g213(.a(\a[29] ), .b(\b[28] ), .out0(new_n309));
  inv000aa1d42x5               g214(.a(new_n309), .o1(new_n310));
  aoi012aa1n03x5               g215(.a(new_n310), .b(new_n306), .c(new_n308), .o1(new_n311));
  aoi112aa1n03x4               g216(.a(new_n309), .b(new_n307), .c(new_n290), .d(new_n305), .o1(new_n312));
  norp02aa1n03x5               g217(.a(new_n311), .b(new_n312), .o1(\s[29] ));
  xorb03aa1n02x5               g218(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano32aa1n02x4               g219(.a(new_n310), .b(new_n294), .c(new_n300), .d(new_n293), .out0(new_n315));
  aoai13aa1n02x5               g220(.a(new_n315), .b(new_n299), .c(new_n191), .d(new_n286), .o1(new_n316));
  oao003aa1n12x5               g221(.a(\a[29] ), .b(\b[28] ), .c(new_n308), .carry(new_n317));
  tech160nm_fixorc02aa1n03p5x5 g222(.a(\a[30] ), .b(\b[29] ), .out0(new_n318));
  inv000aa1d42x5               g223(.a(new_n318), .o1(new_n319));
  aoi012aa1n03x5               g224(.a(new_n319), .b(new_n316), .c(new_n317), .o1(new_n320));
  inv000aa1d42x5               g225(.a(new_n317), .o1(new_n321));
  aoi112aa1n03x4               g226(.a(new_n318), .b(new_n321), .c(new_n290), .d(new_n315), .o1(new_n322));
  norp02aa1n03x5               g227(.a(new_n320), .b(new_n322), .o1(\s[30] ));
  xnrc02aa1n02x5               g228(.a(\b[30] ), .b(\a[31] ), .out0(new_n324));
  inv000aa1d42x5               g229(.a(new_n324), .o1(new_n325));
  and003aa1n02x5               g230(.a(new_n305), .b(new_n318), .c(new_n309), .o(new_n326));
  oaoi03aa1n12x5               g231(.a(\a[30] ), .b(\b[29] ), .c(new_n317), .o1(new_n327));
  aoi112aa1n02x7               g232(.a(new_n325), .b(new_n327), .c(new_n290), .d(new_n326), .o1(new_n328));
  aoai13aa1n04x5               g233(.a(new_n326), .b(new_n299), .c(new_n191), .d(new_n286), .o1(new_n329));
  inv000aa1n02x5               g234(.a(new_n327), .o1(new_n330));
  aoi012aa1n03x5               g235(.a(new_n324), .b(new_n329), .c(new_n330), .o1(new_n331));
  nor002aa1n02x5               g236(.a(new_n331), .b(new_n328), .o1(\s[31] ));
  xnbna2aa1n03x5               g237(.a(new_n111), .b(new_n105), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g238(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n334));
  xorb03aa1n02x5               g239(.a(new_n334), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g240(.a(new_n114), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g241(.a(\a[5] ), .b(\b[4] ), .o(new_n337));
  aob012aa1n02x5               g242(.a(new_n114), .b(\b[4] ), .c(\a[5] ), .out0(new_n338));
  xobna2aa1n03x5               g243(.a(new_n120), .b(new_n338), .c(new_n337), .out0(\s[6] ));
  aobi12aa1n02x5               g244(.a(new_n125), .b(new_n114), .c(new_n132), .out0(new_n340));
  xnrb03aa1n02x5               g245(.a(new_n340), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g246(.a(\a[7] ), .b(\b[6] ), .c(new_n340), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g248(.a(new_n135), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


