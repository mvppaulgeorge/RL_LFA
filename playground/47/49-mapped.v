// Benchmark "adder" written by ABC on Thu Jul 18 12:33:07 2024

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
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n336, new_n337,
    new_n338, new_n340, new_n342, new_n344, new_n345, new_n346, new_n347,
    new_n348, new_n350, new_n352, new_n354, new_n355;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nor042aa1d18x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  inv040aa1n03x5               g003(.a(new_n98), .o1(new_n99));
  oao003aa1n12x5               g004(.a(\a[4] ), .b(\b[3] ), .c(new_n99), .carry(new_n100));
  xorc02aa1n12x5               g005(.a(\a[4] ), .b(\b[3] ), .out0(new_n101));
  and002aa1n12x5               g006(.a(\b[0] ), .b(\a[1] ), .o(new_n102));
  oaoi03aa1n09x5               g007(.a(\a[2] ), .b(\b[1] ), .c(new_n102), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  norb02aa1n06x5               g009(.a(new_n104), .b(new_n98), .out0(new_n105));
  nanp03aa1d12x5               g010(.a(new_n103), .b(new_n101), .c(new_n105), .o1(new_n106));
  nanp02aa1n04x5               g011(.a(new_n106), .b(new_n100), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\a[7] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[6] ), .o1(new_n109));
  aoi022aa1d24x5               g014(.a(\b[5] ), .b(\a[6] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n110));
  oai122aa1n12x5               g015(.a(new_n110), .b(new_n108), .c(new_n109), .d(\a[6] ), .e(\b[5] ), .o1(new_n111));
  xorc02aa1n12x5               g016(.a(\a[8] ), .b(\b[7] ), .out0(new_n112));
  inv000aa1d42x5               g017(.a(\a[5] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[4] ), .o1(new_n114));
  nor042aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  tech160nm_fiaoi012aa1n03p5x5 g020(.a(new_n115), .b(new_n113), .c(new_n114), .o1(new_n116));
  nano22aa1n03x7               g021(.a(new_n111), .b(new_n112), .c(new_n116), .out0(new_n117));
  inv000aa1d42x5               g022(.a(\a[8] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[7] ), .o1(new_n119));
  inv040aa1n02x5               g024(.a(new_n115), .o1(new_n120));
  oai022aa1d24x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  aoi022aa1d24x5               g026(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n122));
  nanp02aa1n06x5               g027(.a(new_n121), .b(new_n122), .o1(new_n123));
  nand22aa1n03x5               g028(.a(new_n123), .b(new_n120), .o1(new_n124));
  oaib12aa1n02x5               g029(.a(new_n124), .b(new_n119), .c(\a[8] ), .out0(new_n125));
  oaib12aa1n02x5               g030(.a(new_n125), .b(\b[7] ), .c(new_n118), .out0(new_n126));
  xorc02aa1n12x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n126), .c(new_n107), .d(new_n117), .o1(new_n128));
  xorc02aa1n12x5               g033(.a(\a[10] ), .b(\b[9] ), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n128), .c(new_n97), .out0(\s[10] ));
  nor002aa1n16x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1d28x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  nanb03aa1n12x5               g038(.a(new_n111), .b(new_n116), .c(new_n112), .out0(new_n134));
  tech160nm_fioaoi03aa1n03p5x5 g039(.a(new_n118), .b(new_n119), .c(new_n124), .o1(new_n135));
  aoai13aa1n12x5               g040(.a(new_n135), .b(new_n134), .c(new_n100), .d(new_n106), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  oai022aa1d18x5               g042(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n138));
  aoai13aa1n02x5               g043(.a(new_n137), .b(new_n138), .c(new_n136), .d(new_n127), .o1(new_n139));
  nano22aa1n02x4               g044(.a(new_n131), .b(new_n137), .c(new_n132), .out0(new_n140));
  aoai13aa1n06x5               g045(.a(new_n140), .b(new_n138), .c(new_n136), .d(new_n127), .o1(new_n141));
  aobi12aa1n02x5               g046(.a(new_n141), .b(new_n139), .c(new_n133), .out0(\s[11] ));
  inv000aa1d42x5               g047(.a(new_n131), .o1(new_n143));
  nor002aa1n04x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand42aa1d28x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n141), .c(new_n143), .out0(\s[12] ));
  nanp02aa1n02x5               g052(.a(new_n107), .b(new_n117), .o1(new_n148));
  nano23aa1n06x5               g053(.a(new_n131), .b(new_n144), .c(new_n145), .d(new_n132), .out0(new_n149));
  nand23aa1d12x5               g054(.a(new_n149), .b(new_n127), .c(new_n129), .o1(new_n150));
  nanp03aa1n03x5               g055(.a(new_n138), .b(new_n137), .c(new_n132), .o1(new_n151));
  nona23aa1n06x5               g056(.a(new_n151), .b(new_n145), .c(new_n131), .d(new_n144), .out0(new_n152));
  and002aa1n06x5               g057(.a(new_n152), .b(new_n145), .o(new_n153));
  inv040aa1n03x5               g058(.a(new_n153), .o1(new_n154));
  aoai13aa1n02x5               g059(.a(new_n154), .b(new_n150), .c(new_n148), .d(new_n135), .o1(new_n155));
  xnrc02aa1n12x5               g060(.a(\b[12] ), .b(\a[13] ), .out0(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n150), .o1(new_n158));
  aoi112aa1n02x5               g063(.a(new_n157), .b(new_n153), .c(new_n136), .d(new_n158), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n159), .b(new_n155), .c(new_n157), .o1(\s[13] ));
  inv040aa1d32x5               g065(.a(\a[13] ), .o1(new_n161));
  inv040aa1d28x5               g066(.a(\b[12] ), .o1(new_n162));
  nand02aa1d24x5               g067(.a(new_n162), .b(new_n161), .o1(new_n163));
  aoai13aa1n03x5               g068(.a(new_n157), .b(new_n153), .c(new_n136), .d(new_n158), .o1(new_n164));
  tech160nm_fixnrc02aa1n02p5x5 g069(.a(\b[13] ), .b(\a[14] ), .out0(new_n165));
  xobna2aa1n03x5               g070(.a(new_n165), .b(new_n164), .c(new_n163), .out0(\s[14] ));
  nor042aa1n06x5               g071(.a(new_n165), .b(new_n156), .o1(new_n167));
  aoai13aa1n06x5               g072(.a(new_n167), .b(new_n153), .c(new_n136), .d(new_n158), .o1(new_n168));
  oaoi03aa1n12x5               g073(.a(\a[14] ), .b(\b[13] ), .c(new_n163), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  xnrc02aa1n12x5               g075(.a(\b[14] ), .b(\a[15] ), .out0(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n168), .c(new_n170), .out0(\s[15] ));
  aoai13aa1n02x5               g078(.a(new_n172), .b(new_n169), .c(new_n155), .d(new_n167), .o1(new_n174));
  xnrc02aa1n12x5               g079(.a(\b[15] ), .b(\a[16] ), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  nor002aa1d32x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n175), .b(new_n177), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n177), .o1(new_n179));
  aoai13aa1n02x7               g084(.a(new_n179), .b(new_n171), .c(new_n168), .d(new_n170), .o1(new_n180));
  aoi022aa1n03x5               g085(.a(new_n180), .b(new_n176), .c(new_n174), .d(new_n178), .o1(\s[16] ));
  nor042aa1n03x5               g086(.a(new_n175), .b(new_n171), .o1(new_n182));
  nano22aa1n12x5               g087(.a(new_n150), .b(new_n167), .c(new_n182), .out0(new_n183));
  aoai13aa1n06x5               g088(.a(new_n183), .b(new_n126), .c(new_n107), .d(new_n117), .o1(new_n184));
  nor042aa1n02x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  aoi022aa1n06x5               g090(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n186));
  norb02aa1n03x5               g091(.a(new_n186), .b(new_n185), .out0(new_n187));
  oai022aa1n02x5               g092(.a(new_n161), .b(new_n162), .c(\b[13] ), .d(\a[14] ), .o1(new_n188));
  aoi012aa1n02x5               g093(.a(new_n177), .b(\a[14] ), .c(\b[13] ), .o1(new_n189));
  nano32aa1n03x7               g094(.a(new_n188), .b(new_n189), .c(new_n145), .d(new_n163), .out0(new_n190));
  oa0012aa1n03x5               g095(.a(new_n186), .b(new_n169), .c(new_n177), .o(new_n191));
  aoi113aa1n06x5               g096(.a(new_n191), .b(new_n185), .c(new_n152), .d(new_n190), .e(new_n187), .o1(new_n192));
  nand02aa1d08x5               g097(.a(new_n184), .b(new_n192), .o1(new_n193));
  xorc02aa1n12x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  oabi12aa1n02x5               g099(.a(new_n194), .b(\a[16] ), .c(\b[15] ), .out0(new_n195));
  aoi113aa1n02x5               g100(.a(new_n191), .b(new_n195), .c(new_n152), .d(new_n190), .e(new_n187), .o1(new_n196));
  aoi022aa1n02x5               g101(.a(new_n193), .b(new_n194), .c(new_n184), .d(new_n196), .o1(\s[17] ));
  nor002aa1d32x5               g102(.a(\b[16] ), .b(\a[17] ), .o1(new_n198));
  inv000aa1n06x5               g103(.a(new_n198), .o1(new_n199));
  nanp03aa1n03x5               g104(.a(new_n152), .b(new_n190), .c(new_n187), .o1(new_n200));
  oaoi13aa1n04x5               g105(.a(new_n185), .b(new_n186), .c(new_n169), .d(new_n177), .o1(new_n201));
  nand02aa1n04x5               g106(.a(new_n200), .b(new_n201), .o1(new_n202));
  aoai13aa1n03x5               g107(.a(new_n194), .b(new_n202), .c(new_n136), .d(new_n183), .o1(new_n203));
  nor002aa1n20x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nand22aa1n12x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  norb02aa1n06x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n203), .c(new_n199), .out0(\s[18] ));
  and002aa1n02x5               g112(.a(new_n194), .b(new_n206), .o(new_n208));
  aoai13aa1n03x5               g113(.a(new_n208), .b(new_n202), .c(new_n136), .d(new_n183), .o1(new_n209));
  oaoi03aa1n02x5               g114(.a(\a[18] ), .b(\b[17] ), .c(new_n199), .o1(new_n210));
  inv000aa1n02x5               g115(.a(new_n210), .o1(new_n211));
  nor002aa1d32x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nand02aa1n12x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n12x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g121(.a(new_n214), .b(new_n210), .c(new_n193), .d(new_n208), .o1(new_n217));
  nor002aa1d32x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand02aa1d28x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  norb02aa1n03x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  inv000aa1d42x5               g125(.a(\a[19] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\b[18] ), .o1(new_n222));
  aboi22aa1n03x5               g127(.a(new_n218), .b(new_n219), .c(new_n221), .d(new_n222), .out0(new_n223));
  inv030aa1n04x5               g128(.a(new_n212), .o1(new_n224));
  inv040aa1n03x5               g129(.a(new_n214), .o1(new_n225));
  aoai13aa1n02x7               g130(.a(new_n224), .b(new_n225), .c(new_n209), .d(new_n211), .o1(new_n226));
  aoi022aa1n03x5               g131(.a(new_n226), .b(new_n220), .c(new_n217), .d(new_n223), .o1(\s[20] ));
  nano32aa1n03x7               g132(.a(new_n225), .b(new_n194), .c(new_n220), .d(new_n206), .out0(new_n228));
  aoai13aa1n03x5               g133(.a(new_n228), .b(new_n202), .c(new_n136), .d(new_n183), .o1(new_n229));
  nanb03aa1n09x5               g134(.a(new_n218), .b(new_n219), .c(new_n213), .out0(new_n230));
  oai112aa1n06x5               g135(.a(new_n224), .b(new_n205), .c(new_n204), .d(new_n198), .o1(new_n231));
  aoi012aa1n12x5               g136(.a(new_n218), .b(new_n212), .c(new_n219), .o1(new_n232));
  oai012aa1n18x5               g137(.a(new_n232), .b(new_n231), .c(new_n230), .o1(new_n233));
  nor002aa1d32x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  nand02aa1d08x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  norb02aa1d27x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n233), .c(new_n193), .d(new_n228), .o1(new_n237));
  nano22aa1n02x5               g142(.a(new_n218), .b(new_n213), .c(new_n219), .out0(new_n238));
  oai012aa1n03x5               g143(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .o1(new_n239));
  oab012aa1n02x5               g144(.a(new_n239), .b(new_n198), .c(new_n204), .out0(new_n240));
  inv020aa1n02x5               g145(.a(new_n232), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(new_n241), .b(new_n236), .c(new_n240), .d(new_n238), .o1(new_n242));
  aobi12aa1n03x7               g147(.a(new_n237), .b(new_n242), .c(new_n229), .out0(\s[21] ));
  nor042aa1n06x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  nand02aa1n16x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n245), .b(new_n244), .out0(new_n246));
  aoib12aa1n02x5               g151(.a(new_n234), .b(new_n245), .c(new_n244), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n233), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n234), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n236), .o1(new_n250));
  aoai13aa1n02x7               g155(.a(new_n249), .b(new_n250), .c(new_n229), .d(new_n248), .o1(new_n251));
  aoi022aa1n03x5               g156(.a(new_n251), .b(new_n246), .c(new_n237), .d(new_n247), .o1(\s[22] ));
  inv000aa1n02x5               g157(.a(new_n228), .o1(new_n253));
  nano22aa1n03x7               g158(.a(new_n253), .b(new_n236), .c(new_n246), .out0(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n202), .c(new_n136), .d(new_n183), .o1(new_n255));
  nano23aa1n06x5               g160(.a(new_n234), .b(new_n244), .c(new_n245), .d(new_n235), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n241), .c(new_n240), .d(new_n238), .o1(new_n257));
  aoi012aa1n09x5               g162(.a(new_n244), .b(new_n234), .c(new_n245), .o1(new_n258));
  nand42aa1n02x5               g163(.a(new_n257), .b(new_n258), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n259), .c(new_n193), .d(new_n254), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n258), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(new_n260), .b(new_n262), .c(new_n233), .d(new_n256), .o1(new_n263));
  aobi12aa1n02x7               g168(.a(new_n261), .b(new_n263), .c(new_n255), .out0(\s[23] ));
  xorc02aa1n03x5               g169(.a(\a[24] ), .b(\b[23] ), .out0(new_n265));
  nor042aa1n09x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  norp02aa1n02x5               g171(.a(new_n265), .b(new_n266), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n259), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n266), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n260), .o1(new_n270));
  aoai13aa1n03x5               g175(.a(new_n269), .b(new_n270), .c(new_n255), .d(new_n268), .o1(new_n271));
  aoi022aa1n02x7               g176(.a(new_n271), .b(new_n265), .c(new_n261), .d(new_n267), .o1(\s[24] ));
  and002aa1n12x5               g177(.a(new_n265), .b(new_n260), .o(new_n273));
  nano22aa1n03x7               g178(.a(new_n253), .b(new_n273), .c(new_n256), .out0(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n202), .c(new_n136), .d(new_n183), .o1(new_n275));
  inv000aa1n06x5               g180(.a(new_n273), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[24] ), .b(\b[23] ), .c(new_n269), .carry(new_n277));
  aoai13aa1n12x5               g182(.a(new_n277), .b(new_n276), .c(new_n257), .d(new_n258), .o1(new_n278));
  xorc02aa1n12x5               g183(.a(\a[25] ), .b(\b[24] ), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n278), .c(new_n193), .d(new_n274), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n273), .b(new_n262), .c(new_n233), .d(new_n256), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n279), .o1(new_n282));
  and003aa1n02x5               g187(.a(new_n281), .b(new_n282), .c(new_n277), .o(new_n283));
  aobi12aa1n03x7               g188(.a(new_n280), .b(new_n283), .c(new_n275), .out0(\s[25] ));
  tech160nm_fixorc02aa1n02p5x5 g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  nor042aa1n06x5               g190(.a(\b[24] ), .b(\a[25] ), .o1(new_n286));
  norp02aa1n02x5               g191(.a(new_n285), .b(new_n286), .o1(new_n287));
  inv000aa1n02x5               g192(.a(new_n278), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n286), .o1(new_n289));
  aoai13aa1n02x7               g194(.a(new_n289), .b(new_n282), .c(new_n275), .d(new_n288), .o1(new_n290));
  aoi022aa1n02x7               g195(.a(new_n290), .b(new_n285), .c(new_n280), .d(new_n287), .o1(\s[26] ));
  and002aa1n12x5               g196(.a(new_n285), .b(new_n279), .o(new_n292));
  nano32aa1d12x5               g197(.a(new_n253), .b(new_n292), .c(new_n256), .d(new_n273), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n202), .c(new_n136), .d(new_n183), .o1(new_n294));
  inv000aa1n02x5               g199(.a(new_n292), .o1(new_n295));
  oao003aa1n06x5               g200(.a(\a[26] ), .b(\b[25] ), .c(new_n289), .carry(new_n296));
  aoai13aa1n04x5               g201(.a(new_n296), .b(new_n295), .c(new_n281), .d(new_n277), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n297), .c(new_n193), .d(new_n293), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n296), .o1(new_n300));
  aoi112aa1n02x5               g205(.a(new_n298), .b(new_n300), .c(new_n278), .d(new_n292), .o1(new_n301));
  aobi12aa1n02x7               g206(.a(new_n299), .b(new_n301), .c(new_n294), .out0(\s[27] ));
  tech160nm_fixorc02aa1n03p5x5 g207(.a(\a[28] ), .b(\b[27] ), .out0(new_n303));
  nor042aa1n03x5               g208(.a(\b[26] ), .b(\a[27] ), .o1(new_n304));
  norp02aa1n02x5               g209(.a(new_n303), .b(new_n304), .o1(new_n305));
  aoi012aa1n09x5               g210(.a(new_n300), .b(new_n278), .c(new_n292), .o1(new_n306));
  inv000aa1n03x5               g211(.a(new_n304), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n298), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n307), .b(new_n308), .c(new_n306), .d(new_n294), .o1(new_n309));
  aoi022aa1n03x5               g214(.a(new_n309), .b(new_n303), .c(new_n299), .d(new_n305), .o1(\s[28] ));
  and002aa1n02x5               g215(.a(new_n303), .b(new_n298), .o(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n297), .c(new_n193), .d(new_n293), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n311), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[28] ), .b(\b[27] ), .c(new_n307), .carry(new_n314));
  aoai13aa1n02x7               g219(.a(new_n314), .b(new_n313), .c(new_n306), .d(new_n294), .o1(new_n315));
  tech160nm_fixorc02aa1n03p5x5 g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  norb02aa1n02x5               g221(.a(new_n314), .b(new_n316), .out0(new_n317));
  aoi022aa1n03x5               g222(.a(new_n315), .b(new_n316), .c(new_n312), .d(new_n317), .o1(\s[29] ));
  xnrb03aa1n02x5               g223(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g224(.a(new_n308), .b(new_n303), .c(new_n316), .out0(new_n320));
  aoai13aa1n02x5               g225(.a(new_n320), .b(new_n297), .c(new_n193), .d(new_n293), .o1(new_n321));
  inv000aa1n02x5               g226(.a(new_n320), .o1(new_n322));
  oao003aa1n03x5               g227(.a(\a[29] ), .b(\b[28] ), .c(new_n314), .carry(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n322), .c(new_n306), .d(new_n294), .o1(new_n324));
  tech160nm_fixorc02aa1n04x5   g229(.a(\a[30] ), .b(\b[29] ), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n323), .b(new_n325), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n321), .d(new_n326), .o1(\s[30] ));
  nano32aa1n02x5               g232(.a(new_n308), .b(new_n325), .c(new_n303), .d(new_n316), .out0(new_n328));
  aoai13aa1n02x5               g233(.a(new_n328), .b(new_n297), .c(new_n193), .d(new_n293), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[31] ), .b(\b[30] ), .out0(new_n330));
  oao003aa1n02x5               g235(.a(\a[30] ), .b(\b[29] ), .c(new_n323), .carry(new_n331));
  norb02aa1n02x5               g236(.a(new_n331), .b(new_n330), .out0(new_n332));
  inv000aa1n02x5               g237(.a(new_n328), .o1(new_n333));
  aoai13aa1n02x7               g238(.a(new_n331), .b(new_n333), .c(new_n306), .d(new_n294), .o1(new_n334));
  aoi022aa1n03x5               g239(.a(new_n334), .b(new_n330), .c(new_n329), .d(new_n332), .o1(\s[31] ));
  orn002aa1n02x5               g240(.a(\a[2] ), .b(\b[1] ), .o(new_n336));
  nanp02aa1n02x5               g241(.a(\b[1] ), .b(\a[2] ), .o1(new_n337));
  nanb03aa1n02x5               g242(.a(new_n102), .b(new_n336), .c(new_n337), .out0(new_n338));
  xnbna2aa1n03x5               g243(.a(new_n105), .b(new_n338), .c(new_n336), .out0(\s[3] ));
  nanp02aa1n02x5               g244(.a(new_n103), .b(new_n105), .o1(new_n340));
  xnbna2aa1n03x5               g245(.a(new_n101), .b(new_n340), .c(new_n99), .out0(\s[4] ));
  xorc02aa1n02x5               g246(.a(\a[5] ), .b(\b[4] ), .out0(new_n342));
  xnbna2aa1n03x5               g247(.a(new_n342), .b(new_n106), .c(new_n100), .out0(\s[5] ));
  xorc02aa1n02x5               g248(.a(\a[6] ), .b(\b[5] ), .out0(new_n344));
  oaoi03aa1n02x5               g249(.a(new_n113), .b(new_n114), .c(new_n107), .o1(new_n345));
  nanp02aa1n02x5               g250(.a(\b[5] ), .b(\a[6] ), .o1(new_n346));
  norb02aa1n02x5               g251(.a(new_n346), .b(new_n121), .out0(new_n347));
  aob012aa1n03x5               g252(.a(new_n347), .b(new_n107), .c(new_n342), .out0(new_n348));
  oai012aa1n02x5               g253(.a(new_n348), .b(new_n345), .c(new_n344), .o1(\s[6] ));
  xorc02aa1n02x5               g254(.a(\a[7] ), .b(\b[6] ), .out0(new_n350));
  xobna2aa1n03x5               g255(.a(new_n350), .b(new_n348), .c(new_n346), .out0(\s[7] ));
  nanp03aa1n02x5               g256(.a(new_n348), .b(new_n346), .c(new_n350), .o1(new_n352));
  xnbna2aa1n03x5               g257(.a(new_n112), .b(new_n352), .c(new_n120), .out0(\s[8] ));
  oabi12aa1n02x5               g258(.a(new_n127), .b(\a[8] ), .c(\b[7] ), .out0(new_n354));
  norb02aa1n02x5               g259(.a(new_n125), .b(new_n354), .out0(new_n355));
  aoi022aa1n02x5               g260(.a(new_n136), .b(new_n127), .c(new_n355), .d(new_n148), .o1(\s[9] ));
endmodule


