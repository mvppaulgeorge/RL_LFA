// Benchmark "adder" written by ABC on Thu Jul 18 07:40:39 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n330, new_n332, new_n333, new_n334,
    new_n336, new_n337;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nor002aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi012aa1n09x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  nand02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n08x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  tech160nm_finor002aa1n05x5   g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n02x4               g012(.a(new_n104), .b(new_n107), .c(new_n106), .d(new_n105), .out0(new_n108));
  aoi012aa1n02x7               g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oai012aa1n09x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  norp02aa1n04x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand22aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor022aa1n04x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nona23aa1n02x5               g019(.a(new_n113), .b(new_n111), .c(new_n114), .d(new_n112), .out0(new_n115));
  nand22aa1n03x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nor002aa1n04x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand42aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  norp02aa1n24x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nona23aa1n03x5               g024(.a(new_n118), .b(new_n116), .c(new_n119), .d(new_n117), .out0(new_n120));
  nor042aa1n02x5               g025(.a(new_n120), .b(new_n115), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n118), .b(new_n119), .c(new_n117), .o1(new_n122));
  oai012aa1n02x5               g027(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n123));
  oai012aa1n06x5               g028(.a(new_n122), .b(new_n120), .c(new_n123), .o1(new_n124));
  xorc02aa1n02x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n03x5               g030(.a(new_n125), .b(new_n124), .c(new_n110), .d(new_n121), .o1(new_n126));
  nor002aa1d32x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand42aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g035(.a(new_n127), .o1(new_n131));
  and002aa1n12x5               g036(.a(\b[9] ), .b(\a[10] ), .o(new_n132));
  aoai13aa1n06x5               g037(.a(new_n131), .b(new_n132), .c(new_n126), .d(new_n99), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nand02aa1d24x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor002aa1d32x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nor002aa1n20x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand02aa1n16x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  aoi112aa1n02x5               g044(.a(new_n139), .b(new_n136), .c(new_n133), .d(new_n135), .o1(new_n140));
  aoai13aa1n03x5               g045(.a(new_n139), .b(new_n136), .c(new_n133), .d(new_n135), .o1(new_n141));
  norb02aa1n03x4               g046(.a(new_n141), .b(new_n140), .out0(\s[12] ));
  and002aa1n02x5               g047(.a(\b[8] ), .b(\a[9] ), .o(new_n143));
  inv000aa1d42x5               g048(.a(new_n135), .o1(new_n144));
  aoi112aa1n09x5               g049(.a(new_n132), .b(new_n127), .c(new_n97), .d(new_n98), .o1(new_n145));
  norb03aa1n09x5               g050(.a(new_n138), .b(new_n136), .c(new_n137), .out0(new_n146));
  nona23aa1d18x5               g051(.a(new_n145), .b(new_n146), .c(new_n144), .d(new_n143), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n124), .c(new_n110), .d(new_n121), .o1(new_n149));
  nona23aa1n09x5               g054(.a(new_n135), .b(new_n138), .c(new_n137), .d(new_n136), .out0(new_n150));
  tech160nm_fioai012aa1n03p5x5 g055(.a(new_n138), .b(new_n137), .c(new_n136), .o1(new_n151));
  aoai13aa1n04x5               g056(.a(new_n128), .b(new_n127), .c(new_n97), .d(new_n98), .o1(new_n152));
  oai012aa1n18x5               g057(.a(new_n151), .b(new_n150), .c(new_n152), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  nanp02aa1n04x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nor022aa1n16x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  norb02aa1n03x5               g061(.a(new_n155), .b(new_n156), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n149), .c(new_n154), .out0(\s[13] ));
  orn002aa1n02x5               g063(.a(\a[13] ), .b(\b[12] ), .o(new_n159));
  inv000aa1n02x5               g064(.a(new_n103), .o1(new_n160));
  nano23aa1n02x4               g065(.a(new_n106), .b(new_n105), .c(new_n107), .d(new_n104), .out0(new_n161));
  aobi12aa1n03x7               g066(.a(new_n109), .b(new_n161), .c(new_n160), .out0(new_n162));
  nano23aa1n02x4               g067(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n163));
  nanb02aa1n02x5               g068(.a(new_n115), .b(new_n163), .out0(new_n164));
  oabi12aa1n03x5               g069(.a(new_n124), .b(new_n162), .c(new_n164), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n157), .b(new_n153), .c(new_n165), .d(new_n148), .o1(new_n166));
  norp02aa1n12x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand02aa1d06x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  norb02aa1n03x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n166), .c(new_n159), .out0(\s[14] ));
  nona23aa1n09x5               g075(.a(new_n155), .b(new_n168), .c(new_n167), .d(new_n156), .out0(new_n171));
  aoi012aa1n02x7               g076(.a(new_n167), .b(new_n156), .c(new_n168), .o1(new_n172));
  aoai13aa1n04x5               g077(.a(new_n172), .b(new_n171), .c(new_n149), .d(new_n154), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nanp02aa1n04x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nor002aa1n12x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nor042aa1n06x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanp02aa1n04x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n177), .b(new_n178), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n179), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n180), .b(new_n176), .c(new_n173), .d(new_n175), .o1(new_n181));
  aoai13aa1n03x5               g086(.a(new_n180), .b(new_n176), .c(new_n173), .d(new_n175), .o1(new_n182));
  norb02aa1n03x4               g087(.a(new_n182), .b(new_n181), .out0(\s[16] ));
  nano23aa1n06x5               g088(.a(new_n177), .b(new_n176), .c(new_n178), .d(new_n175), .out0(new_n184));
  nano32aa1d12x5               g089(.a(new_n147), .b(new_n184), .c(new_n157), .d(new_n169), .out0(new_n185));
  aoai13aa1n12x5               g090(.a(new_n185), .b(new_n124), .c(new_n110), .d(new_n121), .o1(new_n186));
  nona23aa1n09x5               g091(.a(new_n175), .b(new_n178), .c(new_n177), .d(new_n176), .out0(new_n187));
  nor022aa1n02x5               g092(.a(new_n187), .b(new_n171), .o1(new_n188));
  nor022aa1n02x5               g093(.a(new_n187), .b(new_n172), .o1(new_n189));
  oa0012aa1n02x5               g094(.a(new_n178), .b(new_n177), .c(new_n176), .o(new_n190));
  aoi112aa1n09x5               g095(.a(new_n190), .b(new_n189), .c(new_n153), .d(new_n188), .o1(new_n191));
  nand02aa1d08x5               g096(.a(new_n186), .b(new_n191), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[16] ), .o1(new_n195));
  oaoi03aa1n03x5               g100(.a(new_n194), .b(new_n195), .c(new_n192), .o1(new_n196));
  xnrb03aa1n03x5               g101(.a(new_n196), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n02x5               g102(.a(new_n195), .b(new_n194), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  nor022aa1n04x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nand42aa1n03x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nanb02aa1n03x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  nano22aa1n03x7               g107(.a(new_n202), .b(new_n198), .c(new_n199), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n201), .b(new_n200), .c(new_n194), .d(new_n195), .o1(new_n205));
  aoai13aa1n04x5               g110(.a(new_n205), .b(new_n204), .c(new_n186), .d(new_n191), .o1(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n03x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nor042aa1n09x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nor042aa1n02x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nand42aa1n03x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nanb02aa1n02x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoi112aa1n03x4               g119(.a(new_n214), .b(new_n210), .c(new_n206), .d(new_n209), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n210), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n209), .b(new_n210), .out0(new_n217));
  nand02aa1n02x5               g122(.a(new_n206), .b(new_n217), .o1(new_n218));
  tech160nm_fiaoi012aa1n02p5x5 g123(.a(new_n213), .b(new_n218), .c(new_n216), .o1(new_n219));
  norp02aa1n03x5               g124(.a(new_n219), .b(new_n215), .o1(\s[20] ));
  nano23aa1n09x5               g125(.a(new_n211), .b(new_n210), .c(new_n212), .d(new_n209), .out0(new_n221));
  nanp02aa1n02x5               g126(.a(new_n203), .b(new_n221), .o1(new_n222));
  inv000aa1n02x5               g127(.a(new_n205), .o1(new_n223));
  oaih12aa1n02x5               g128(.a(new_n212), .b(new_n211), .c(new_n210), .o1(new_n224));
  aobi12aa1n06x5               g129(.a(new_n224), .b(new_n221), .c(new_n223), .out0(new_n225));
  aoai13aa1n04x5               g130(.a(new_n225), .b(new_n222), .c(new_n186), .d(new_n191), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nanp02aa1n02x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  nor042aa1n06x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[22] ), .b(\b[21] ), .out0(new_n230));
  aoi112aa1n03x4               g135(.a(new_n230), .b(new_n229), .c(new_n226), .d(new_n228), .o1(new_n231));
  inv000aa1n02x5               g136(.a(new_n229), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n228), .b(new_n229), .out0(new_n233));
  nand02aa1n02x5               g138(.a(new_n226), .b(new_n233), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n230), .o1(new_n235));
  tech160nm_fiaoi012aa1n03p5x5 g140(.a(new_n235), .b(new_n234), .c(new_n232), .o1(new_n236));
  nor042aa1n03x5               g141(.a(new_n236), .b(new_n231), .o1(\s[22] ));
  norp02aa1n02x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  nano23aa1n06x5               g144(.a(new_n238), .b(new_n229), .c(new_n239), .d(new_n228), .out0(new_n240));
  nand23aa1n03x5               g145(.a(new_n203), .b(new_n221), .c(new_n240), .o1(new_n241));
  nona23aa1n02x4               g146(.a(new_n209), .b(new_n212), .c(new_n211), .d(new_n210), .out0(new_n242));
  tech160nm_fioai012aa1n05x5   g147(.a(new_n224), .b(new_n242), .c(new_n205), .o1(new_n243));
  oaoi03aa1n03x5               g148(.a(\a[22] ), .b(\b[21] ), .c(new_n232), .o1(new_n244));
  aoi012aa1n02x5               g149(.a(new_n244), .b(new_n243), .c(new_n240), .o1(new_n245));
  aoai13aa1n04x5               g150(.a(new_n245), .b(new_n241), .c(new_n186), .d(new_n191), .o1(new_n246));
  xorb03aa1n02x5               g151(.a(new_n246), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n16x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  nanp02aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  nor002aa1n03x5               g154(.a(\b[23] ), .b(\a[24] ), .o1(new_n250));
  nand02aa1n03x5               g155(.a(\b[23] ), .b(\a[24] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n250), .out0(new_n252));
  aoi112aa1n03x4               g157(.a(new_n248), .b(new_n252), .c(new_n246), .d(new_n249), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n248), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n249), .b(new_n248), .out0(new_n255));
  nand02aa1n02x5               g160(.a(new_n246), .b(new_n255), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n252), .o1(new_n257));
  tech160nm_fiaoi012aa1n03p5x5 g162(.a(new_n257), .b(new_n256), .c(new_n254), .o1(new_n258));
  nor042aa1n03x5               g163(.a(new_n258), .b(new_n253), .o1(\s[24] ));
  nano23aa1n06x5               g164(.a(new_n248), .b(new_n250), .c(new_n251), .d(new_n249), .out0(new_n260));
  nanb03aa1n02x5               g165(.a(new_n222), .b(new_n260), .c(new_n240), .out0(new_n261));
  nona22aa1n02x4               g166(.a(new_n251), .b(new_n250), .c(new_n248), .out0(new_n262));
  aoi022aa1n06x5               g167(.a(new_n260), .b(new_n244), .c(new_n262), .d(new_n251), .o1(new_n263));
  inv020aa1n02x5               g168(.a(new_n263), .o1(new_n264));
  aoi013aa1n02x4               g169(.a(new_n264), .b(new_n243), .c(new_n240), .d(new_n260), .o1(new_n265));
  aoai13aa1n04x5               g170(.a(new_n265), .b(new_n261), .c(new_n186), .d(new_n191), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nanp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  nor042aa1n04x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  xnrc02aa1n12x5               g174(.a(\b[25] ), .b(\a[26] ), .out0(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoi112aa1n03x4               g176(.a(new_n271), .b(new_n269), .c(new_n266), .d(new_n268), .o1(new_n272));
  inv000aa1n02x5               g177(.a(new_n269), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n268), .b(new_n269), .out0(new_n274));
  nanp02aa1n03x5               g179(.a(new_n266), .b(new_n274), .o1(new_n275));
  tech160nm_fiaoi012aa1n03p5x5 g180(.a(new_n270), .b(new_n275), .c(new_n273), .o1(new_n276));
  norp02aa1n03x5               g181(.a(new_n276), .b(new_n272), .o1(\s[26] ));
  nanp02aa1n02x5               g182(.a(new_n153), .b(new_n188), .o1(new_n278));
  nona22aa1n02x4               g183(.a(new_n278), .b(new_n189), .c(new_n190), .out0(new_n279));
  nano22aa1n06x5               g184(.a(new_n270), .b(new_n268), .c(new_n273), .out0(new_n280));
  nano22aa1n03x7               g185(.a(new_n241), .b(new_n260), .c(new_n280), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n279), .c(new_n165), .d(new_n185), .o1(new_n282));
  nano22aa1n03x7               g187(.a(new_n225), .b(new_n240), .c(new_n260), .out0(new_n283));
  oaoi03aa1n02x5               g188(.a(\a[26] ), .b(\b[25] ), .c(new_n273), .o1(new_n284));
  oaoi13aa1n09x5               g189(.a(new_n284), .b(new_n280), .c(new_n283), .d(new_n264), .o1(new_n285));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  nanp02aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  norb02aa1n02x5               g192(.a(new_n287), .b(new_n286), .out0(new_n288));
  xnbna2aa1n03x5               g193(.a(new_n288), .b(new_n282), .c(new_n285), .out0(\s[27] ));
  inv000aa1n06x5               g194(.a(new_n286), .o1(new_n290));
  aobi12aa1n02x7               g195(.a(new_n288), .b(new_n282), .c(new_n285), .out0(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[27] ), .b(\a[28] ), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n291), .b(new_n290), .c(new_n292), .out0(new_n293));
  nanp03aa1n02x5               g198(.a(new_n243), .b(new_n240), .c(new_n260), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n280), .o1(new_n295));
  inv000aa1n02x5               g200(.a(new_n284), .o1(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n295), .c(new_n294), .d(new_n263), .o1(new_n297));
  aoai13aa1n03x5               g202(.a(new_n288), .b(new_n297), .c(new_n192), .d(new_n281), .o1(new_n298));
  tech160nm_fiaoi012aa1n03p5x5 g203(.a(new_n292), .b(new_n298), .c(new_n290), .o1(new_n299));
  norp02aa1n03x5               g204(.a(new_n299), .b(new_n293), .o1(\s[28] ));
  xnrc02aa1n02x5               g205(.a(\b[28] ), .b(\a[29] ), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n292), .b(new_n290), .c(new_n287), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n297), .c(new_n192), .d(new_n281), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[28] ), .b(\b[27] ), .c(new_n290), .carry(new_n304));
  tech160nm_fiaoi012aa1n03p5x5 g209(.a(new_n301), .b(new_n303), .c(new_n304), .o1(new_n305));
  aobi12aa1n02x7               g210(.a(new_n302), .b(new_n282), .c(new_n285), .out0(new_n306));
  nano22aa1n02x4               g211(.a(new_n306), .b(new_n301), .c(new_n304), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n305), .b(new_n307), .o1(\s[29] ));
  xorb03aa1n02x5               g213(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g214(.a(new_n301), .b(new_n292), .c(new_n287), .d(new_n290), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n297), .c(new_n192), .d(new_n281), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[29] ), .b(\b[28] ), .c(new_n304), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[29] ), .b(\a[30] ), .out0(new_n313));
  tech160nm_fiaoi012aa1n03p5x5 g218(.a(new_n313), .b(new_n311), .c(new_n312), .o1(new_n314));
  aobi12aa1n02x7               g219(.a(new_n310), .b(new_n282), .c(new_n285), .out0(new_n315));
  nano22aa1n02x4               g220(.a(new_n315), .b(new_n312), .c(new_n313), .out0(new_n316));
  norp02aa1n03x5               g221(.a(new_n314), .b(new_n316), .o1(\s[30] ));
  norb03aa1n02x5               g222(.a(new_n302), .b(new_n301), .c(new_n313), .out0(new_n318));
  aobi12aa1n02x7               g223(.a(new_n318), .b(new_n282), .c(new_n285), .out0(new_n319));
  oao003aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .c(new_n312), .carry(new_n320));
  xnrc02aa1n02x5               g225(.a(\b[30] ), .b(\a[31] ), .out0(new_n321));
  nano22aa1n02x4               g226(.a(new_n319), .b(new_n320), .c(new_n321), .out0(new_n322));
  aoai13aa1n03x5               g227(.a(new_n318), .b(new_n297), .c(new_n192), .d(new_n281), .o1(new_n323));
  tech160nm_fiaoi012aa1n03p5x5 g228(.a(new_n321), .b(new_n323), .c(new_n320), .o1(new_n324));
  norp02aa1n03x5               g229(.a(new_n324), .b(new_n322), .o1(\s[31] ));
  xnrb03aa1n02x5               g230(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g231(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g233(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g234(.a(\a[5] ), .b(\b[4] ), .c(new_n162), .o1(new_n330));
  xorb03aa1n02x5               g235(.a(new_n330), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g236(.a(new_n116), .b(new_n119), .out0(new_n332));
  aoai13aa1n02x5               g237(.a(new_n332), .b(new_n114), .c(new_n330), .d(new_n113), .o1(new_n333));
  aoi112aa1n02x5               g238(.a(new_n332), .b(new_n114), .c(new_n330), .d(new_n113), .o1(new_n334));
  norb02aa1n02x5               g239(.a(new_n333), .b(new_n334), .out0(\s[7] ));
  norb02aa1n02x5               g240(.a(new_n118), .b(new_n117), .out0(new_n336));
  inv000aa1d42x5               g241(.a(new_n119), .o1(new_n337));
  xnbna2aa1n03x5               g242(.a(new_n336), .b(new_n333), .c(new_n337), .out0(\s[8] ));
  xorb03aa1n02x5               g243(.a(new_n165), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


