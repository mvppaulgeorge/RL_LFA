// Benchmark "adder" written by ABC on Wed Jul 17 18:08:54 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n314, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n337,
    new_n338, new_n340, new_n341, new_n343, new_n344, new_n347, new_n348,
    new_n349, new_n351, new_n353;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  oai112aa1n06x5               g002(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nor002aa1d32x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nanp02aa1n04x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  aoi113aa1n02x5               g007(.a(new_n102), .b(new_n100), .c(new_n98), .d(new_n101), .e(new_n99), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor042aa1d18x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  nano22aa1n03x7               g011(.a(new_n105), .b(new_n104), .c(new_n106), .out0(new_n107));
  inv040aa1d32x5               g012(.a(\a[8] ), .o1(new_n108));
  inv030aa1d32x5               g013(.a(\b[7] ), .o1(new_n109));
  nand02aa1n04x5               g014(.a(new_n109), .b(new_n108), .o1(new_n110));
  nand42aa1n04x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand02aa1d08x5               g016(.a(new_n110), .b(new_n111), .o1(new_n112));
  inv000aa1d42x5               g017(.a(new_n112), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n08x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nano23aa1n02x4               g022(.a(new_n114), .b(new_n116), .c(new_n117), .d(new_n115), .out0(new_n118));
  nanp03aa1n02x5               g023(.a(new_n118), .b(new_n113), .c(new_n107), .o1(new_n119));
  inv040aa1n08x5               g024(.a(new_n105), .o1(new_n120));
  oaoi03aa1n12x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  inv000aa1d42x5               g026(.a(new_n114), .o1(new_n122));
  nano22aa1n03x7               g027(.a(new_n112), .b(new_n122), .c(new_n115), .out0(new_n123));
  oaoi03aa1n09x5               g028(.a(new_n108), .b(new_n109), .c(new_n114), .o1(new_n124));
  inv000aa1d42x5               g029(.a(new_n124), .o1(new_n125));
  aoi012aa1n12x5               g030(.a(new_n125), .b(new_n123), .c(new_n121), .o1(new_n126));
  oai012aa1n06x5               g031(.a(new_n126), .b(new_n119), .c(new_n103), .o1(new_n127));
  tech160nm_fixorc02aa1n04x5   g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  nor002aa1n03x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand02aa1n06x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n03x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  aoai13aa1n04x5               g036(.a(new_n131), .b(new_n97), .c(new_n127), .d(new_n128), .o1(new_n132));
  aoi112aa1n02x5               g037(.a(new_n131), .b(new_n97), .c(new_n127), .d(new_n128), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n132), .b(new_n133), .out0(\s[10] ));
  aoi012aa1d18x5               g039(.a(new_n129), .b(new_n97), .c(new_n130), .o1(new_n135));
  nor042aa1n04x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand42aa1n03x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n132), .c(new_n135), .out0(\s[11] ));
  aob012aa1n03x5               g044(.a(new_n138), .b(new_n132), .c(new_n135), .out0(new_n140));
  nor002aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n09x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  aoib12aa1n02x5               g048(.a(new_n136), .b(new_n142), .c(new_n141), .out0(new_n144));
  oai012aa1n02x5               g049(.a(new_n140), .b(\b[10] ), .c(\a[11] ), .o1(new_n145));
  aoi022aa1n02x5               g050(.a(new_n145), .b(new_n143), .c(new_n140), .d(new_n144), .o1(\s[12] ));
  nona23aa1n09x5               g051(.a(new_n142), .b(new_n137), .c(new_n136), .d(new_n141), .out0(new_n147));
  nano22aa1n03x7               g052(.a(new_n147), .b(new_n128), .c(new_n131), .out0(new_n148));
  nanp02aa1n02x5               g053(.a(new_n127), .b(new_n148), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n135), .o1(new_n150));
  nano23aa1n03x7               g055(.a(new_n136), .b(new_n141), .c(new_n142), .d(new_n137), .out0(new_n151));
  aoi012aa1d18x5               g056(.a(new_n141), .b(new_n136), .c(new_n142), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n153), .b(new_n151), .c(new_n150), .o1(new_n154));
  nanp02aa1n03x5               g059(.a(new_n149), .b(new_n154), .o1(new_n155));
  nor002aa1d32x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  tech160nm_finand02aa1n05x5   g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nanb02aa1d24x5               g062(.a(new_n156), .b(new_n157), .out0(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  aoi112aa1n02x5               g064(.a(new_n153), .b(new_n159), .c(new_n151), .d(new_n150), .o1(new_n160));
  aoi022aa1n02x5               g065(.a(new_n155), .b(new_n159), .c(new_n149), .d(new_n160), .o1(\s[13] ));
  inv000aa1d42x5               g066(.a(new_n156), .o1(new_n162));
  oai012aa1n12x5               g067(.a(new_n152), .b(new_n147), .c(new_n135), .o1(new_n163));
  aoai13aa1n02x5               g068(.a(new_n159), .b(new_n163), .c(new_n127), .d(new_n148), .o1(new_n164));
  nor042aa1n03x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nand22aa1n04x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nanb02aa1n03x5               g071(.a(new_n165), .b(new_n166), .out0(new_n167));
  xobna2aa1n03x5               g072(.a(new_n167), .b(new_n164), .c(new_n162), .out0(\s[14] ));
  nano23aa1n03x7               g073(.a(new_n156), .b(new_n165), .c(new_n166), .d(new_n157), .out0(new_n169));
  aoai13aa1n02x5               g074(.a(new_n169), .b(new_n163), .c(new_n127), .d(new_n148), .o1(new_n170));
  aoi012aa1n12x5               g075(.a(new_n165), .b(new_n156), .c(new_n166), .o1(new_n171));
  nor042aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand42aa1n03x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n170), .c(new_n171), .out0(\s[15] ));
  inv000aa1d42x5               g080(.a(new_n171), .o1(new_n176));
  aoai13aa1n06x5               g081(.a(new_n174), .b(new_n176), .c(new_n155), .d(new_n169), .o1(new_n177));
  nor042aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand42aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  aoib12aa1n02x5               g085(.a(new_n172), .b(new_n179), .c(new_n178), .out0(new_n181));
  oai012aa1n06x5               g086(.a(new_n177), .b(\b[14] ), .c(\a[15] ), .o1(new_n182));
  aoi022aa1n02x5               g087(.a(new_n182), .b(new_n180), .c(new_n177), .d(new_n181), .o1(\s[16] ));
  nand23aa1n03x5               g088(.a(new_n169), .b(new_n174), .c(new_n180), .o1(new_n184));
  nano32aa1n02x5               g089(.a(new_n184), .b(new_n151), .c(new_n131), .d(new_n128), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n127), .b(new_n185), .o1(new_n186));
  inv000aa1n02x5               g091(.a(new_n98), .o1(new_n187));
  nanb03aa1n06x5               g092(.a(new_n100), .b(new_n101), .c(new_n99), .out0(new_n188));
  norp02aa1n02x5               g093(.a(new_n102), .b(new_n100), .o1(new_n189));
  tech160nm_fioai012aa1n03p5x5 g094(.a(new_n189), .b(new_n188), .c(new_n187), .o1(new_n190));
  nanb02aa1d24x5               g095(.a(new_n114), .b(new_n115), .out0(new_n191));
  nanb02aa1n02x5               g096(.a(new_n116), .b(new_n117), .out0(new_n192));
  nor043aa1n03x5               g097(.a(new_n192), .b(new_n191), .c(new_n112), .o1(new_n193));
  nand23aa1n03x5               g098(.a(new_n190), .b(new_n193), .c(new_n107), .o1(new_n194));
  nona23aa1n09x5               g099(.a(new_n179), .b(new_n173), .c(new_n172), .d(new_n178), .out0(new_n195));
  nor043aa1n06x5               g100(.a(new_n195), .b(new_n167), .c(new_n158), .o1(new_n196));
  nand02aa1d04x5               g101(.a(new_n148), .b(new_n196), .o1(new_n197));
  aoi012aa1n02x5               g102(.a(new_n178), .b(new_n172), .c(new_n179), .o1(new_n198));
  tech160nm_fioai012aa1n04x5   g103(.a(new_n198), .b(new_n195), .c(new_n171), .o1(new_n199));
  aoi012aa1n06x5               g104(.a(new_n199), .b(new_n163), .c(new_n196), .o1(new_n200));
  aoai13aa1n12x5               g105(.a(new_n200), .b(new_n197), .c(new_n194), .d(new_n126), .o1(new_n201));
  tech160nm_fixorc02aa1n02p5x5 g106(.a(\a[17] ), .b(\b[16] ), .out0(new_n202));
  norp02aa1n02x5               g107(.a(new_n195), .b(new_n171), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n198), .out0(new_n204));
  aoi112aa1n02x5               g109(.a(new_n204), .b(new_n203), .c(new_n163), .d(new_n196), .o1(new_n205));
  aoi022aa1n02x5               g110(.a(new_n201), .b(new_n202), .c(new_n186), .d(new_n205), .o1(\s[17] ));
  inv000aa1d42x5               g111(.a(\a[17] ), .o1(new_n207));
  nanb02aa1n02x5               g112(.a(\b[16] ), .b(new_n207), .out0(new_n208));
  oabi12aa1n02x5               g113(.a(new_n199), .b(new_n154), .c(new_n184), .out0(new_n209));
  aoai13aa1n02x5               g114(.a(new_n202), .b(new_n209), .c(new_n127), .d(new_n185), .o1(new_n210));
  xorc02aa1n12x5               g115(.a(\a[18] ), .b(\b[17] ), .out0(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n210), .c(new_n208), .out0(\s[18] ));
  inv000aa1d42x5               g117(.a(\a[18] ), .o1(new_n213));
  xroi22aa1d04x5               g118(.a(new_n207), .b(\b[16] ), .c(new_n213), .d(\b[17] ), .out0(new_n214));
  oaoi03aa1n02x5               g119(.a(\a[18] ), .b(\b[17] ), .c(new_n208), .o1(new_n215));
  nor042aa1n04x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  nanp02aa1n04x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  norb02aa1n06x5               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  aoai13aa1n04x5               g123(.a(new_n218), .b(new_n215), .c(new_n201), .d(new_n214), .o1(new_n219));
  aoi112aa1n02x5               g124(.a(new_n218), .b(new_n215), .c(new_n201), .d(new_n214), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n219), .b(new_n220), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nand42aa1n06x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  norb02aa1n06x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  aoib12aa1n02x5               g130(.a(new_n216), .b(new_n224), .c(new_n223), .out0(new_n226));
  nanb02aa1n03x5               g131(.a(new_n216), .b(new_n219), .out0(new_n227));
  aoi022aa1n02x5               g132(.a(new_n227), .b(new_n225), .c(new_n219), .d(new_n226), .o1(\s[20] ));
  nano23aa1n02x5               g133(.a(new_n216), .b(new_n223), .c(new_n224), .d(new_n217), .out0(new_n229));
  nand23aa1n06x5               g134(.a(new_n229), .b(new_n202), .c(new_n211), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nor042aa1n02x5               g136(.a(\b[17] ), .b(\a[18] ), .o1(new_n232));
  aoi112aa1n09x5               g137(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n233));
  oai112aa1n06x5               g138(.a(new_n218), .b(new_n225), .c(new_n233), .d(new_n232), .o1(new_n234));
  tech160nm_fioai012aa1n04x5   g139(.a(new_n224), .b(new_n223), .c(new_n216), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(new_n234), .b(new_n235), .o1(new_n236));
  xorc02aa1n02x5               g141(.a(\a[21] ), .b(\b[20] ), .out0(new_n237));
  aoai13aa1n04x5               g142(.a(new_n237), .b(new_n236), .c(new_n201), .d(new_n231), .o1(new_n238));
  nano22aa1n02x4               g143(.a(new_n237), .b(new_n234), .c(new_n235), .out0(new_n239));
  aobi12aa1n02x5               g144(.a(new_n239), .b(new_n201), .c(new_n231), .out0(new_n240));
  norb02aa1n02x5               g145(.a(new_n238), .b(new_n240), .out0(\s[21] ));
  xorc02aa1n02x5               g146(.a(\a[22] ), .b(\b[21] ), .out0(new_n242));
  norp02aa1n02x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  norp02aa1n02x5               g148(.a(new_n242), .b(new_n243), .o1(new_n244));
  inv000aa1d42x5               g149(.a(\a[21] ), .o1(new_n245));
  oaib12aa1n02x5               g150(.a(new_n238), .b(\b[20] ), .c(new_n245), .out0(new_n246));
  aoi022aa1n02x5               g151(.a(new_n246), .b(new_n242), .c(new_n238), .d(new_n244), .o1(\s[22] ));
  nand22aa1n03x5               g152(.a(new_n242), .b(new_n237), .o1(new_n248));
  nona22aa1n06x5               g153(.a(new_n201), .b(new_n230), .c(new_n248), .out0(new_n249));
  inv000aa1d42x5               g154(.a(\a[22] ), .o1(new_n250));
  oai022aa1n02x5               g155(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n251));
  oaib12aa1n02x5               g156(.a(new_n251), .b(new_n250), .c(\b[21] ), .out0(new_n252));
  aoai13aa1n12x5               g157(.a(new_n252), .b(new_n248), .c(new_n234), .d(new_n235), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[23] ), .b(\b[22] ), .out0(new_n255));
  aob012aa1n03x5               g160(.a(new_n255), .b(new_n249), .c(new_n254), .out0(new_n256));
  xroi22aa1d04x5               g161(.a(new_n245), .b(\b[20] ), .c(new_n250), .d(\b[21] ), .out0(new_n257));
  nanb02aa1n02x5               g162(.a(new_n255), .b(new_n252), .out0(new_n258));
  aoi012aa1n02x5               g163(.a(new_n258), .b(new_n236), .c(new_n257), .o1(new_n259));
  aobi12aa1n02x7               g164(.a(new_n256), .b(new_n259), .c(new_n249), .out0(\s[23] ));
  xorc02aa1n02x5               g165(.a(\a[24] ), .b(\b[23] ), .out0(new_n261));
  nor002aa1d24x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  norp02aa1n02x5               g167(.a(new_n261), .b(new_n262), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n262), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n255), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n264), .b(new_n265), .c(new_n249), .d(new_n254), .o1(new_n266));
  aoi022aa1n03x5               g171(.a(new_n266), .b(new_n261), .c(new_n256), .d(new_n263), .o1(\s[24] ));
  nanp02aa1n02x5               g172(.a(\b[22] ), .b(\a[23] ), .o1(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[23] ), .b(\a[24] ), .out0(new_n269));
  nano22aa1n02x5               g174(.a(new_n269), .b(new_n264), .c(new_n268), .out0(new_n270));
  nano22aa1n02x4               g175(.a(new_n230), .b(new_n257), .c(new_n270), .out0(new_n271));
  oaoi03aa1n02x5               g176(.a(\a[24] ), .b(\b[23] ), .c(new_n264), .o1(new_n272));
  tech160nm_fiao0012aa1n02p5x5 g177(.a(new_n272), .b(new_n253), .c(new_n270), .o(new_n273));
  xorc02aa1n02x5               g178(.a(\a[25] ), .b(\b[24] ), .out0(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n273), .c(new_n201), .d(new_n271), .o1(new_n275));
  aoi112aa1n02x5               g180(.a(new_n274), .b(new_n272), .c(new_n253), .d(new_n270), .o1(new_n276));
  aobi12aa1n02x5               g181(.a(new_n276), .b(new_n271), .c(new_n201), .out0(new_n277));
  norb02aa1n02x5               g182(.a(new_n275), .b(new_n277), .out0(\s[25] ));
  xorc02aa1n02x5               g183(.a(\a[26] ), .b(\b[25] ), .out0(new_n279));
  norp02aa1n02x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n279), .b(new_n280), .o1(new_n281));
  inv000aa1n03x5               g186(.a(new_n280), .o1(new_n282));
  nanp02aa1n02x5               g187(.a(new_n275), .b(new_n282), .o1(new_n283));
  aoi022aa1n02x5               g188(.a(new_n283), .b(new_n279), .c(new_n275), .d(new_n281), .o1(\s[26] ));
  and002aa1n02x5               g189(.a(new_n279), .b(new_n274), .o(new_n285));
  aoai13aa1n12x5               g190(.a(new_n285), .b(new_n272), .c(new_n253), .d(new_n270), .o1(new_n286));
  nano32aa1n03x7               g191(.a(new_n230), .b(new_n285), .c(new_n257), .d(new_n270), .out0(new_n287));
  aoai13aa1n06x5               g192(.a(new_n287), .b(new_n209), .c(new_n127), .d(new_n185), .o1(new_n288));
  oaoi03aa1n02x5               g193(.a(\a[26] ), .b(\b[25] ), .c(new_n282), .o1(new_n289));
  inv020aa1n02x5               g194(.a(new_n289), .o1(new_n290));
  nanp03aa1d12x5               g195(.a(new_n288), .b(new_n286), .c(new_n290), .o1(new_n291));
  xorc02aa1n12x5               g196(.a(\a[27] ), .b(\b[26] ), .out0(new_n292));
  aoi112aa1n02x5               g197(.a(new_n292), .b(new_n289), .c(new_n201), .d(new_n287), .o1(new_n293));
  aoi022aa1n02x5               g198(.a(new_n293), .b(new_n286), .c(new_n291), .d(new_n292), .o1(\s[27] ));
  nand42aa1n04x5               g199(.a(new_n291), .b(new_n292), .o1(new_n295));
  xorc02aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .out0(new_n296));
  norp02aa1n02x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  norp02aa1n02x5               g202(.a(new_n296), .b(new_n297), .o1(new_n298));
  nanp02aa1n02x5               g203(.a(\b[25] ), .b(\a[26] ), .o1(new_n299));
  oai022aa1n02x5               g204(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n300));
  aoi022aa1n06x5               g205(.a(new_n201), .b(new_n287), .c(new_n299), .d(new_n300), .o1(new_n301));
  inv000aa1n03x5               g206(.a(new_n297), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n292), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n302), .b(new_n303), .c(new_n301), .d(new_n286), .o1(new_n304));
  aoi022aa1n03x5               g209(.a(new_n304), .b(new_n296), .c(new_n295), .d(new_n298), .o1(\s[28] ));
  and002aa1n02x5               g210(.a(new_n296), .b(new_n292), .o(new_n306));
  nand02aa1n03x5               g211(.a(new_n291), .b(new_n306), .o1(new_n307));
  xorc02aa1n02x5               g212(.a(\a[29] ), .b(\b[28] ), .out0(new_n308));
  oao003aa1n02x5               g213(.a(\a[28] ), .b(\b[27] ), .c(new_n302), .carry(new_n309));
  norb02aa1n02x5               g214(.a(new_n309), .b(new_n308), .out0(new_n310));
  inv000aa1d42x5               g215(.a(new_n306), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n309), .b(new_n311), .c(new_n301), .d(new_n286), .o1(new_n312));
  aoi022aa1n03x5               g217(.a(new_n312), .b(new_n308), .c(new_n307), .d(new_n310), .o1(\s[29] ));
  nanp02aa1n02x5               g218(.a(\b[0] ), .b(\a[1] ), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g220(.a(new_n303), .b(new_n296), .c(new_n308), .out0(new_n316));
  inv000aa1d42x5               g221(.a(new_n316), .o1(new_n317));
  aoi013aa1n02x4               g222(.a(new_n317), .b(new_n288), .c(new_n286), .d(new_n290), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[30] ), .b(\b[29] ), .out0(new_n319));
  inv000aa1d42x5               g224(.a(\a[29] ), .o1(new_n320));
  aoib12aa1n02x5               g225(.a(new_n319), .b(new_n320), .c(\b[28] ), .out0(new_n321));
  aoai13aa1n02x5               g226(.a(new_n321), .b(new_n309), .c(\a[29] ), .d(\b[28] ), .o1(new_n322));
  tech160nm_fiaoi012aa1n05x5   g227(.a(new_n322), .b(new_n291), .c(new_n316), .o1(new_n323));
  oaoi03aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .c(new_n309), .o1(new_n324));
  oaoi13aa1n02x7               g229(.a(new_n323), .b(new_n319), .c(new_n324), .d(new_n318), .o1(\s[30] ));
  nanp03aa1n02x5               g230(.a(new_n306), .b(new_n308), .c(new_n319), .o1(new_n326));
  nanb02aa1n02x5               g231(.a(new_n326), .b(new_n291), .out0(new_n327));
  xorc02aa1n02x5               g232(.a(\a[31] ), .b(\b[30] ), .out0(new_n328));
  inv000aa1d42x5               g233(.a(\a[30] ), .o1(new_n329));
  inv000aa1d42x5               g234(.a(\b[29] ), .o1(new_n330));
  aboi22aa1n03x5               g235(.a(\b[28] ), .b(new_n320), .c(new_n330), .d(new_n329), .out0(new_n331));
  aoai13aa1n02x5               g236(.a(new_n331), .b(new_n309), .c(\a[29] ), .d(\b[28] ), .o1(new_n332));
  oaoi13aa1n02x5               g237(.a(new_n328), .b(new_n332), .c(new_n329), .d(new_n330), .o1(new_n333));
  oaib12aa1n02x5               g238(.a(new_n332), .b(new_n330), .c(\a[30] ), .out0(new_n334));
  aoai13aa1n03x5               g239(.a(new_n334), .b(new_n326), .c(new_n301), .d(new_n286), .o1(new_n335));
  aoi022aa1n03x5               g240(.a(new_n335), .b(new_n328), .c(new_n327), .d(new_n333), .o1(\s[31] ));
  inv000aa1d42x5               g241(.a(new_n100), .o1(new_n337));
  aoi022aa1n02x5               g242(.a(new_n98), .b(new_n99), .c(new_n337), .d(new_n101), .o1(new_n338));
  oab012aa1n02x4               g243(.a(new_n338), .b(new_n188), .c(new_n187), .out0(\s[3] ));
  nanb02aa1n02x5               g244(.a(new_n188), .b(new_n98), .out0(new_n340));
  norb02aa1n02x5               g245(.a(new_n104), .b(new_n102), .out0(new_n341));
  xnbna2aa1n03x5               g246(.a(new_n341), .b(new_n340), .c(new_n337), .out0(\s[4] ));
  nanp02aa1n02x5               g247(.a(new_n190), .b(new_n107), .o1(new_n343));
  aoi022aa1n02x5               g248(.a(new_n190), .b(new_n104), .c(new_n106), .d(new_n120), .o1(new_n344));
  norb02aa1n02x5               g249(.a(new_n343), .b(new_n344), .out0(\s[5] ));
  xobna2aa1n03x5               g250(.a(new_n192), .b(new_n343), .c(new_n120), .out0(\s[6] ));
  inv000aa1d42x5               g251(.a(new_n191), .o1(new_n347));
  inv000aa1d42x5               g252(.a(new_n121), .o1(new_n348));
  nanb03aa1n02x5               g253(.a(new_n192), .b(new_n190), .c(new_n107), .out0(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n347), .b(new_n349), .c(new_n348), .out0(\s[7] ));
  aob012aa1n02x5               g255(.a(new_n347), .b(new_n349), .c(new_n348), .out0(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n113), .b(new_n351), .c(new_n122), .out0(\s[8] ));
  aoi112aa1n02x5               g257(.a(new_n125), .b(new_n128), .c(new_n123), .d(new_n121), .o1(new_n353));
  aoi022aa1n02x5               g258(.a(new_n127), .b(new_n128), .c(new_n194), .d(new_n353), .o1(\s[9] ));
endmodule


