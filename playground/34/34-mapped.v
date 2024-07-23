// Benchmark "adder" written by ABC on Thu Jul 18 05:41:16 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n173, new_n174, new_n175, new_n176,
    new_n178, new_n179, new_n180, new_n181, new_n182, new_n183, new_n184,
    new_n185, new_n186, new_n187, new_n189, new_n190, new_n191, new_n193,
    new_n194, new_n195, new_n196, new_n197, new_n198, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n224, new_n225, new_n226, new_n227, new_n228, new_n229, new_n230,
    new_n231, new_n232, new_n233, new_n235, new_n236, new_n237, new_n240,
    new_n241, new_n242, new_n243, new_n244, new_n245, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n264, new_n265, new_n266, new_n267, new_n268, new_n269, new_n271,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n277, new_n278,
    new_n279, new_n280, new_n281, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n303, new_n304, new_n305, new_n306, new_n307, new_n308, new_n309,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n316, new_n317,
    new_n318, new_n319, new_n320, new_n322, new_n323, new_n324, new_n325,
    new_n326, new_n327, new_n328, new_n329, new_n330, new_n332, new_n333,
    new_n334, new_n335, new_n336, new_n337, new_n338, new_n339, new_n342,
    new_n343, new_n344, new_n345, new_n346, new_n347, new_n348, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n355, new_n357, new_n358,
    new_n359, new_n361, new_n364, new_n365, new_n368, new_n369, new_n371,
    new_n372, new_n373;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  and002aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o(new_n98));
  nand42aa1n04x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor002aa1n06x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor042aa1n06x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  norb03aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .out0(new_n102));
  obai22aa1n03x5               g007(.a(new_n99), .b(new_n102), .c(\a[9] ), .d(\b[8] ), .out0(new_n103));
  inv000aa1d42x5               g008(.a(\a[2] ), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\b[1] ), .o1(new_n105));
  nanp02aa1n03x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  oaoi03aa1n02x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  norp02aa1n04x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nand42aa1n02x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  nor002aa1n04x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nand42aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nona23aa1n03x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  tech160nm_fioai012aa1n04x5   g017(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n113));
  oai012aa1n06x5               g018(.a(new_n113), .b(new_n112), .c(new_n107), .o1(new_n114));
  nand42aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n99), .b(new_n115), .c(new_n101), .d(new_n100), .out0(new_n116));
  nor002aa1n04x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  norb02aa1n06x4               g023(.a(new_n118), .b(new_n117), .out0(new_n119));
  nor042aa1n04x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  tech160nm_finand02aa1n05x5   g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  norb02aa1n02x7               g026(.a(new_n121), .b(new_n120), .out0(new_n122));
  nano22aa1n03x7               g027(.a(new_n116), .b(new_n119), .c(new_n122), .out0(new_n123));
  nanb02aa1n02x5               g028(.a(new_n100), .b(new_n99), .out0(new_n124));
  nanb02aa1n02x5               g029(.a(new_n101), .b(new_n115), .out0(new_n125));
  aoi012aa1n02x5               g030(.a(new_n117), .b(new_n120), .c(new_n118), .o1(new_n126));
  norp03aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n124), .o1(new_n127));
  aoi112aa1n03x5               g032(.a(new_n127), .b(new_n103), .c(new_n114), .d(new_n123), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand02aa1n06x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  oai012aa1d24x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  nano22aa1n03x7               g037(.a(new_n128), .b(new_n129), .c(new_n132), .out0(new_n133));
  oaoi13aa1n02x5               g038(.a(new_n133), .b(new_n97), .c(new_n98), .d(new_n128), .o1(\s[10] ));
  inv000aa1d42x5               g039(.a(\a[10] ), .o1(new_n135));
  inv000aa1d42x5               g040(.a(\b[9] ), .o1(new_n136));
  nor002aa1n02x5               g041(.a(\b[8] ), .b(\a[9] ), .o1(new_n137));
  oao003aa1n06x5               g042(.a(new_n135), .b(new_n136), .c(new_n137), .carry(new_n138));
  nor002aa1n03x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand42aa1n06x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n06x4               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  tech160nm_fioai012aa1n05x5   g046(.a(new_n141), .b(new_n133), .c(new_n138), .o1(new_n142));
  norp03aa1n02x5               g047(.a(new_n133), .b(new_n138), .c(new_n141), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n142), .b(new_n143), .out0(\s[11] ));
  oai012aa1n02x5               g049(.a(new_n142), .b(\b[10] ), .c(\a[11] ), .o1(new_n145));
  xorc02aa1n12x5               g050(.a(\a[12] ), .b(\b[11] ), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n145), .b(new_n147), .o1(new_n148));
  nona22aa1n02x4               g053(.a(new_n142), .b(new_n147), .c(new_n139), .out0(new_n149));
  nanp02aa1n02x5               g054(.a(new_n148), .b(new_n149), .o1(\s[12] ));
  nanp02aa1n02x5               g055(.a(new_n105), .b(new_n104), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[1] ), .b(\a[2] ), .o1(new_n152));
  aob012aa1n02x5               g057(.a(new_n151), .b(new_n106), .c(new_n152), .out0(new_n153));
  norb02aa1n02x5               g058(.a(new_n109), .b(new_n108), .out0(new_n154));
  norb02aa1n02x5               g059(.a(new_n111), .b(new_n110), .out0(new_n155));
  nand23aa1n03x5               g060(.a(new_n153), .b(new_n154), .c(new_n155), .o1(new_n156));
  nano23aa1n02x4               g061(.a(new_n117), .b(new_n120), .c(new_n121), .d(new_n118), .out0(new_n157));
  nona22aa1n03x5               g062(.a(new_n157), .b(new_n125), .c(new_n124), .out0(new_n158));
  norb02aa1n02x5               g063(.a(new_n99), .b(new_n100), .out0(new_n159));
  oa0012aa1n03x5               g064(.a(new_n99), .b(new_n100), .c(new_n101), .o(new_n160));
  norb02aa1n02x7               g065(.a(new_n115), .b(new_n101), .out0(new_n161));
  inv000aa1n02x5               g066(.a(new_n117), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(new_n120), .b(new_n118), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(new_n163), .b(new_n162), .o1(new_n164));
  aoi013aa1n06x4               g069(.a(new_n160), .b(new_n164), .c(new_n161), .d(new_n159), .o1(new_n165));
  aoai13aa1n06x5               g070(.a(new_n165), .b(new_n158), .c(new_n156), .d(new_n113), .o1(new_n166));
  oai012aa1n02x5               g071(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .o1(new_n167));
  nona23aa1d18x5               g072(.a(new_n146), .b(new_n141), .c(new_n167), .d(new_n131), .out0(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[11] ), .b(\a[12] ), .o1(new_n170));
  oab012aa1n02x4               g075(.a(new_n139), .b(\a[12] ), .c(\b[11] ), .out0(new_n171));
  aobi12aa1n02x5               g076(.a(new_n171), .b(new_n138), .c(new_n140), .out0(new_n172));
  norb02aa1n02x5               g077(.a(new_n170), .b(new_n172), .out0(new_n173));
  tech160nm_fixorc02aa1n02p5x5 g078(.a(\a[13] ), .b(\b[12] ), .out0(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n173), .c(new_n166), .d(new_n169), .o1(new_n175));
  aoi112aa1n02x5               g080(.a(new_n174), .b(new_n173), .c(new_n166), .d(new_n169), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n175), .b(new_n176), .out0(\s[13] ));
  nor042aa1n02x5               g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\a[13] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[12] ), .o1(new_n180));
  nanp02aa1n04x5               g085(.a(\b[13] ), .b(\a[14] ), .o1(new_n181));
  aboi22aa1n03x5               g086(.a(new_n178), .b(new_n181), .c(new_n179), .d(new_n180), .out0(new_n182));
  norb02aa1n02x5               g087(.a(new_n181), .b(new_n178), .out0(new_n183));
  and002aa1n02x5               g088(.a(new_n174), .b(new_n183), .o(new_n184));
  aoai13aa1n06x5               g089(.a(new_n184), .b(new_n173), .c(new_n166), .d(new_n169), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n181), .b(new_n178), .c(new_n179), .d(new_n180), .o1(new_n186));
  nand42aa1n04x5               g091(.a(new_n185), .b(new_n186), .o1(new_n187));
  aboi22aa1n03x5               g092(.a(new_n178), .b(new_n187), .c(new_n175), .d(new_n182), .out0(\s[14] ));
  nor002aa1n02x5               g093(.a(\b[14] ), .b(\a[15] ), .o1(new_n189));
  nand42aa1n03x5               g094(.a(\b[14] ), .b(\a[15] ), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(new_n189), .b(new_n190), .out0(new_n191));
  xobna2aa1n03x5               g096(.a(new_n191), .b(new_n185), .c(new_n186), .out0(\s[15] ));
  nor042aa1n03x5               g097(.a(\b[15] ), .b(\a[16] ), .o1(new_n193));
  nand42aa1n08x5               g098(.a(\b[15] ), .b(\a[16] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(new_n195));
  aoai13aa1n03x5               g100(.a(new_n195), .b(new_n189), .c(new_n187), .d(new_n190), .o1(new_n196));
  oai022aa1n02x5               g101(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n197));
  aoai13aa1n02x5               g102(.a(new_n194), .b(new_n191), .c(new_n185), .d(new_n186), .o1(new_n198));
  oai012aa1n02x5               g103(.a(new_n196), .b(new_n198), .c(new_n197), .o1(\s[16] ));
  inv000aa1n02x5               g104(.a(new_n160), .o1(new_n200));
  oai013aa1n03x5               g105(.a(new_n200), .b(new_n126), .c(new_n125), .d(new_n124), .o1(new_n201));
  nano23aa1n02x5               g106(.a(new_n189), .b(new_n193), .c(new_n194), .d(new_n190), .out0(new_n202));
  nano32aa1d12x5               g107(.a(new_n168), .b(new_n202), .c(new_n174), .d(new_n183), .out0(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n201), .c(new_n114), .d(new_n123), .o1(new_n204));
  oaoi03aa1n02x5               g109(.a(new_n135), .b(new_n136), .c(new_n137), .o1(new_n205));
  aoai13aa1n02x5               g110(.a(new_n171), .b(new_n205), .c(\a[11] ), .d(\b[10] ), .o1(new_n206));
  nano22aa1n03x7               g111(.a(new_n193), .b(new_n190), .c(new_n194), .out0(new_n207));
  oai012aa1n03x5               g112(.a(new_n181), .b(\b[14] ), .c(\a[15] ), .o1(new_n208));
  oai012aa1n02x5               g113(.a(new_n170), .b(\b[12] ), .c(\a[13] ), .o1(new_n209));
  aoi012aa1n02x5               g114(.a(new_n178), .b(\a[13] ), .c(\b[12] ), .o1(new_n210));
  nona23aa1n02x5               g115(.a(new_n207), .b(new_n210), .c(new_n209), .d(new_n208), .out0(new_n211));
  oai022aa1n02x5               g116(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n212));
  oai112aa1n02x5               g117(.a(new_n212), .b(new_n181), .c(\b[14] ), .d(\a[15] ), .o1(new_n213));
  oai012aa1n02x5               g118(.a(new_n194), .b(new_n193), .c(new_n189), .o1(new_n214));
  oaib12aa1n03x5               g119(.a(new_n214), .b(new_n213), .c(new_n207), .out0(new_n215));
  aoib12aa1n12x5               g120(.a(new_n215), .b(new_n206), .c(new_n211), .out0(new_n216));
  nand42aa1n06x5               g121(.a(new_n204), .b(new_n216), .o1(new_n217));
  nor042aa1n06x5               g122(.a(\b[16] ), .b(\a[17] ), .o1(new_n218));
  nanp02aa1n04x5               g123(.a(\b[16] ), .b(\a[17] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  aboi22aa1n03x5               g125(.a(new_n218), .b(new_n219), .c(new_n197), .d(new_n194), .out0(new_n221));
  nona22aa1n02x4               g126(.a(new_n187), .b(new_n191), .c(new_n195), .out0(new_n222));
  aoi022aa1n02x5               g127(.a(new_n222), .b(new_n221), .c(new_n217), .d(new_n220), .o1(\s[17] ));
  nor042aa1n04x5               g128(.a(\b[17] ), .b(\a[18] ), .o1(new_n224));
  nand02aa1n04x5               g129(.a(\b[17] ), .b(\a[18] ), .o1(new_n225));
  obai22aa1n02x7               g130(.a(new_n225), .b(new_n224), .c(\a[17] ), .d(\b[16] ), .out0(new_n226));
  aoi012aa1n02x5               g131(.a(new_n226), .b(new_n217), .c(new_n220), .o1(new_n227));
  aboi22aa1n03x5               g132(.a(new_n213), .b(new_n207), .c(new_n197), .d(new_n194), .out0(new_n228));
  oaih12aa1n02x5               g133(.a(new_n228), .b(new_n172), .c(new_n211), .o1(new_n229));
  nano23aa1n06x5               g134(.a(new_n218), .b(new_n224), .c(new_n225), .d(new_n219), .out0(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n229), .c(new_n166), .d(new_n203), .o1(new_n231));
  oai012aa1n02x5               g136(.a(new_n225), .b(new_n224), .c(new_n218), .o1(new_n232));
  nand42aa1n03x5               g137(.a(new_n231), .b(new_n232), .o1(new_n233));
  aoib12aa1n02x5               g138(.a(new_n227), .b(new_n233), .c(new_n224), .out0(\s[18] ));
  nor042aa1n02x5               g139(.a(\b[18] ), .b(\a[19] ), .o1(new_n235));
  nand02aa1n03x5               g140(.a(\b[18] ), .b(\a[19] ), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  xnbna2aa1n03x5               g142(.a(new_n237), .b(new_n231), .c(new_n232), .out0(\s[19] ));
  xnrc02aa1n02x5               g143(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g144(.a(\b[19] ), .b(\a[20] ), .o1(new_n240));
  nand02aa1n04x5               g145(.a(\b[19] ), .b(\a[20] ), .o1(new_n241));
  nanb02aa1n02x5               g146(.a(new_n240), .b(new_n241), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n235), .c(new_n233), .d(new_n236), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n237), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n241), .b(new_n244), .c(new_n231), .d(new_n232), .o1(new_n245));
  oai013aa1n02x4               g150(.a(new_n243), .b(new_n245), .c(new_n235), .d(new_n240), .o1(\s[20] ));
  nano23aa1n06x5               g151(.a(new_n235), .b(new_n240), .c(new_n241), .d(new_n236), .out0(new_n247));
  nand42aa1n06x5               g152(.a(new_n247), .b(new_n230), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  nanb03aa1n03x5               g154(.a(new_n240), .b(new_n241), .c(new_n236), .out0(new_n250));
  orn002aa1n02x5               g155(.a(\a[19] ), .b(\b[18] ), .o(new_n251));
  oai112aa1n02x5               g156(.a(new_n251), .b(new_n225), .c(new_n224), .d(new_n218), .o1(new_n252));
  tech160nm_fioai012aa1n03p5x5 g157(.a(new_n241), .b(new_n240), .c(new_n235), .o1(new_n253));
  oai012aa1n06x5               g158(.a(new_n253), .b(new_n252), .c(new_n250), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[21] ), .b(\b[20] ), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n254), .c(new_n217), .d(new_n249), .o1(new_n256));
  norp02aa1n02x5               g161(.a(new_n224), .b(new_n218), .o1(new_n257));
  nano22aa1n02x4               g162(.a(new_n240), .b(new_n236), .c(new_n241), .out0(new_n258));
  oai012aa1n02x5               g163(.a(new_n225), .b(\b[18] ), .c(\a[19] ), .o1(new_n259));
  nona22aa1n02x4               g164(.a(new_n258), .b(new_n257), .c(new_n259), .out0(new_n260));
  nano22aa1n02x4               g165(.a(new_n255), .b(new_n260), .c(new_n253), .out0(new_n261));
  aobi12aa1n02x5               g166(.a(new_n261), .b(new_n217), .c(new_n249), .out0(new_n262));
  norb02aa1n02x7               g167(.a(new_n256), .b(new_n262), .out0(\s[21] ));
  inv000aa1d42x5               g168(.a(\a[21] ), .o1(new_n264));
  nanb02aa1n02x5               g169(.a(\b[20] ), .b(new_n264), .out0(new_n265));
  xorc02aa1n02x5               g170(.a(\a[22] ), .b(\b[21] ), .out0(new_n266));
  and002aa1n02x5               g171(.a(\b[21] ), .b(\a[22] ), .o(new_n267));
  oai022aa1n02x5               g172(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n268));
  nona22aa1n02x5               g173(.a(new_n256), .b(new_n267), .c(new_n268), .out0(new_n269));
  aoai13aa1n03x5               g174(.a(new_n269), .b(new_n266), .c(new_n265), .d(new_n256), .o1(\s[22] ));
  nanp02aa1n02x5               g175(.a(new_n266), .b(new_n255), .o1(new_n271));
  nano22aa1n02x4               g176(.a(new_n271), .b(new_n230), .c(new_n247), .out0(new_n272));
  oaoi03aa1n12x5               g177(.a(\a[22] ), .b(\b[21] ), .c(new_n265), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  aoai13aa1n02x5               g179(.a(new_n274), .b(new_n271), .c(new_n260), .d(new_n253), .o1(new_n275));
  xorc02aa1n02x5               g180(.a(\a[23] ), .b(\b[22] ), .out0(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n275), .c(new_n217), .d(new_n272), .o1(new_n277));
  inv000aa1d42x5               g182(.a(\b[21] ), .o1(new_n278));
  xroi22aa1d04x5               g183(.a(new_n278), .b(\a[22] ), .c(new_n264), .d(\b[20] ), .out0(new_n279));
  aoi112aa1n02x5               g184(.a(new_n276), .b(new_n273), .c(new_n254), .d(new_n279), .o1(new_n280));
  aobi12aa1n02x5               g185(.a(new_n280), .b(new_n217), .c(new_n272), .out0(new_n281));
  norb02aa1n03x4               g186(.a(new_n277), .b(new_n281), .out0(\s[23] ));
  nor042aa1n03x5               g187(.a(\b[22] ), .b(\a[23] ), .o1(new_n283));
  inv000aa1n03x5               g188(.a(new_n283), .o1(new_n284));
  xorc02aa1n02x5               g189(.a(\a[24] ), .b(\b[23] ), .out0(new_n285));
  oai022aa1n02x5               g190(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(\a[24] ), .c(\b[23] ), .o1(new_n287));
  tech160nm_finand02aa1n03p5x5 g192(.a(new_n277), .b(new_n287), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n285), .c(new_n284), .d(new_n277), .o1(\s[24] ));
  nanp02aa1n02x5               g194(.a(\b[22] ), .b(\a[23] ), .o1(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[23] ), .b(\a[24] ), .out0(new_n291));
  nano22aa1n02x4               g196(.a(new_n291), .b(new_n284), .c(new_n290), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n248), .b(new_n279), .c(new_n292), .out0(new_n293));
  aoai13aa1n04x5               g198(.a(new_n292), .b(new_n273), .c(new_n254), .d(new_n279), .o1(new_n294));
  oaoi03aa1n02x5               g199(.a(\a[24] ), .b(\b[23] ), .c(new_n284), .o1(new_n295));
  inv000aa1n02x5               g200(.a(new_n295), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(new_n294), .b(new_n296), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[25] ), .b(\b[24] ), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n297), .c(new_n217), .d(new_n293), .o1(new_n299));
  nona22aa1n02x4               g204(.a(new_n294), .b(new_n295), .c(new_n298), .out0(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n217), .c(new_n293), .o1(new_n301));
  norb02aa1n02x7               g206(.a(new_n299), .b(new_n301), .out0(\s[25] ));
  nor042aa1n02x5               g207(.a(\b[24] ), .b(\a[25] ), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n303), .o1(new_n304));
  xorc02aa1n06x5               g209(.a(\a[26] ), .b(\b[25] ), .out0(new_n305));
  inv000aa1d42x5               g210(.a(\a[26] ), .o1(new_n306));
  inv000aa1d42x5               g211(.a(\b[25] ), .o1(new_n307));
  aoi012aa1n02x5               g212(.a(new_n303), .b(new_n306), .c(new_n307), .o1(new_n308));
  oai112aa1n02x7               g213(.a(new_n299), .b(new_n308), .c(new_n307), .d(new_n306), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n305), .c(new_n304), .d(new_n299), .o1(\s[26] ));
  tech160nm_finand02aa1n03p5x5 g215(.a(new_n305), .b(new_n298), .o1(new_n311));
  nano23aa1n06x5               g216(.a(new_n311), .b(new_n248), .c(new_n279), .d(new_n292), .out0(new_n312));
  aoai13aa1n06x5               g217(.a(new_n312), .b(new_n229), .c(new_n166), .d(new_n203), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n311), .o1(new_n314));
  aoai13aa1n04x5               g219(.a(new_n314), .b(new_n295), .c(new_n275), .d(new_n292), .o1(new_n315));
  oaoi03aa1n02x5               g220(.a(new_n306), .b(new_n307), .c(new_n303), .o1(new_n316));
  nanp03aa1n03x5               g221(.a(new_n313), .b(new_n315), .c(new_n316), .o1(new_n317));
  xorc02aa1n12x5               g222(.a(\a[27] ), .b(\b[26] ), .out0(new_n318));
  inv000aa1d42x5               g223(.a(new_n318), .o1(new_n319));
  and003aa1n02x5               g224(.a(new_n315), .b(new_n319), .c(new_n316), .o(new_n320));
  aoi022aa1n02x5               g225(.a(new_n320), .b(new_n313), .c(new_n317), .d(new_n318), .o1(\s[27] ));
  norp02aa1n02x5               g226(.a(\b[26] ), .b(\a[27] ), .o1(new_n322));
  inv000aa1d42x5               g227(.a(new_n322), .o1(new_n323));
  aobi12aa1n06x5               g228(.a(new_n312), .b(new_n204), .c(new_n216), .out0(new_n324));
  aoai13aa1n04x5               g229(.a(new_n316), .b(new_n311), .c(new_n294), .d(new_n296), .o1(new_n325));
  oaih12aa1n02x5               g230(.a(new_n318), .b(new_n325), .c(new_n324), .o1(new_n326));
  xorc02aa1n02x5               g231(.a(\a[28] ), .b(\b[27] ), .out0(new_n327));
  oai022aa1n02x5               g232(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n328));
  aoi012aa1n02x5               g233(.a(new_n328), .b(\a[28] ), .c(\b[27] ), .o1(new_n329));
  tech160nm_finand02aa1n03p5x5 g234(.a(new_n326), .b(new_n329), .o1(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n327), .c(new_n326), .d(new_n323), .o1(\s[28] ));
  xorc02aa1n02x5               g236(.a(\a[29] ), .b(\b[28] ), .out0(new_n332));
  and002aa1n02x5               g237(.a(new_n327), .b(new_n318), .o(new_n333));
  oaih12aa1n02x5               g238(.a(new_n333), .b(new_n325), .c(new_n324), .o1(new_n334));
  inv000aa1d42x5               g239(.a(\b[27] ), .o1(new_n335));
  oaib12aa1n09x5               g240(.a(new_n328), .b(new_n335), .c(\a[28] ), .out0(new_n336));
  nand43aa1n03x5               g241(.a(new_n334), .b(new_n336), .c(new_n332), .o1(new_n337));
  inv000aa1d42x5               g242(.a(new_n336), .o1(new_n338));
  oaoi13aa1n03x5               g243(.a(new_n338), .b(new_n333), .c(new_n325), .d(new_n324), .o1(new_n339));
  oaih12aa1n02x5               g244(.a(new_n337), .b(new_n339), .c(new_n332), .o1(\s[29] ));
  xorb03aa1n02x5               g245(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g246(.a(new_n319), .b(new_n327), .c(new_n332), .out0(new_n342));
  oaoi03aa1n02x5               g247(.a(\a[29] ), .b(\b[28] ), .c(new_n336), .o1(new_n343));
  oaoi13aa1n03x5               g248(.a(new_n343), .b(new_n342), .c(new_n325), .d(new_n324), .o1(new_n344));
  xorc02aa1n02x5               g249(.a(\a[30] ), .b(\b[29] ), .out0(new_n345));
  oaih12aa1n02x5               g250(.a(new_n342), .b(new_n325), .c(new_n324), .o1(new_n346));
  norb02aa1n02x5               g251(.a(new_n345), .b(new_n343), .out0(new_n347));
  tech160nm_finand02aa1n03p5x5 g252(.a(new_n346), .b(new_n347), .o1(new_n348));
  oaih12aa1n02x5               g253(.a(new_n348), .b(new_n344), .c(new_n345), .o1(\s[30] ));
  nano32aa1n02x4               g254(.a(new_n319), .b(new_n345), .c(new_n327), .d(new_n332), .out0(new_n350));
  aoi012aa1n02x5               g255(.a(new_n347), .b(\a[30] ), .c(\b[29] ), .o1(new_n351));
  xnrc02aa1n02x5               g256(.a(\b[30] ), .b(\a[31] ), .out0(new_n352));
  aoai13aa1n03x5               g257(.a(new_n352), .b(new_n351), .c(new_n317), .d(new_n350), .o1(new_n353));
  oaih12aa1n02x5               g258(.a(new_n350), .b(new_n325), .c(new_n324), .o1(new_n354));
  nona22aa1n02x5               g259(.a(new_n354), .b(new_n351), .c(new_n352), .out0(new_n355));
  nanp02aa1n03x5               g260(.a(new_n353), .b(new_n355), .o1(\s[31] ));
  nanp03aa1n02x5               g261(.a(new_n151), .b(new_n106), .c(new_n152), .o1(new_n357));
  aoi012aa1n02x5               g262(.a(new_n110), .b(new_n104), .c(new_n105), .o1(new_n358));
  nanp03aa1n02x5               g263(.a(new_n357), .b(new_n111), .c(new_n358), .o1(new_n359));
  oai012aa1n02x5               g264(.a(new_n359), .b(new_n155), .c(new_n107), .o1(\s[3] ));
  aoi012aa1n02x5               g265(.a(new_n154), .b(new_n359), .c(new_n111), .o1(new_n361));
  aoib12aa1n02x5               g266(.a(new_n361), .b(new_n114), .c(new_n108), .out0(\s[4] ));
  xnbna2aa1n03x5               g267(.a(new_n122), .b(new_n156), .c(new_n113), .out0(\s[5] ));
  aoai13aa1n02x5               g268(.a(new_n119), .b(new_n120), .c(new_n114), .d(new_n121), .o1(new_n364));
  aoi112aa1n02x5               g269(.a(new_n120), .b(new_n119), .c(new_n114), .d(new_n122), .o1(new_n365));
  norb02aa1n02x5               g270(.a(new_n364), .b(new_n365), .out0(\s[6] ));
  xnbna2aa1n03x5               g271(.a(new_n161), .b(new_n364), .c(new_n162), .out0(\s[7] ));
  aoi012aa1n02x5               g272(.a(new_n125), .b(new_n364), .c(new_n162), .o1(new_n368));
  oai012aa1n02x5               g273(.a(new_n124), .b(new_n368), .c(new_n101), .o1(new_n369));
  oaib12aa1n02x5               g274(.a(new_n369), .b(new_n368), .c(new_n102), .out0(\s[8] ));
  nanp02aa1n02x5               g275(.a(new_n114), .b(new_n123), .o1(new_n371));
  norb02aa1n02x5               g276(.a(new_n130), .b(new_n137), .out0(new_n372));
  aoi113aa1n02x5               g277(.a(new_n160), .b(new_n372), .c(new_n164), .d(new_n161), .e(new_n159), .o1(new_n373));
  aoi022aa1n02x5               g278(.a(new_n166), .b(new_n372), .c(new_n371), .d(new_n373), .o1(\s[9] ));
endmodule


