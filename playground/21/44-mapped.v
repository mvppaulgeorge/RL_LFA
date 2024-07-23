// Benchmark "adder" written by ABC on Wed Jul 17 23:07:54 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n318, new_n319, new_n321, new_n324, new_n325, new_n327,
    new_n329;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n04x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n06x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1d32x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  nanp02aa1n04x5               g006(.a(\b[8] ), .b(\a[9] ), .o1(new_n102));
  nor002aa1n20x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  nand02aa1n04x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  norp02aa1n04x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  inv000aa1d42x5               g012(.a(\b[5] ), .o1(new_n108));
  nanb02aa1n12x5               g013(.a(\a[6] ), .b(new_n108), .out0(new_n109));
  nand02aa1n06x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  oai112aa1n06x5               g015(.a(new_n109), .b(new_n110), .c(\b[4] ), .d(\a[5] ), .o1(new_n111));
  aoi112aa1n06x5               g016(.a(new_n107), .b(new_n111), .c(\a[5] ), .d(\b[4] ), .o1(new_n112));
  and002aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o(new_n113));
  nand02aa1n03x5               g018(.a(\b[0] ), .b(\a[1] ), .o1(new_n114));
  nand42aa1n04x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nor042aa1n02x5               g020(.a(\b[1] ), .b(\a[2] ), .o1(new_n116));
  norb03aa1n03x5               g021(.a(new_n115), .b(new_n114), .c(new_n116), .out0(new_n117));
  norp02aa1n12x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nanp02aa1n04x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nanb03aa1n06x5               g024(.a(new_n118), .b(new_n119), .c(new_n115), .out0(new_n120));
  oab012aa1n02x4               g025(.a(new_n118), .b(\a[4] ), .c(\b[3] ), .out0(new_n121));
  oaoi13aa1n02x7               g026(.a(new_n113), .b(new_n121), .c(new_n117), .d(new_n120), .o1(new_n122));
  aoi112aa1n06x5               g027(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n123));
  nano23aa1n02x4               g028(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n124));
  nanp03aa1n02x5               g029(.a(new_n124), .b(new_n110), .c(new_n111), .o1(new_n125));
  nona22aa1n02x4               g030(.a(new_n125), .b(new_n123), .c(new_n103), .out0(new_n126));
  aoai13aa1n03x5               g031(.a(new_n102), .b(new_n126), .c(new_n112), .d(new_n122), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n99), .b(new_n127), .c(new_n101), .out0(\s[10] ));
  and002aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o(new_n129));
  nona32aa1n03x5               g034(.a(new_n127), .b(new_n100), .c(new_n129), .d(new_n97), .out0(new_n130));
  nor002aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1n08x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n06x4               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xobna2aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n98), .out0(\s[11] ));
  nor002aa1n06x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nand02aa1d04x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n06x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoi013aa1n03x5               g042(.a(new_n131), .b(new_n130), .c(new_n98), .d(new_n132), .o1(new_n138));
  xnrc02aa1n03x5               g043(.a(new_n138), .b(new_n137), .out0(\s[12] ));
  inv000aa1n02x5               g044(.a(new_n113), .o1(new_n140));
  oaih12aa1n02x5               g045(.a(new_n121), .b(new_n117), .c(new_n120), .o1(new_n141));
  nanp03aa1n06x5               g046(.a(new_n112), .b(new_n141), .c(new_n140), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n103), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n123), .o1(new_n144));
  nano22aa1n03x5               g049(.a(new_n107), .b(new_n111), .c(new_n110), .out0(new_n145));
  nano22aa1n03x7               g050(.a(new_n145), .b(new_n143), .c(new_n144), .out0(new_n146));
  nanp02aa1n06x5               g051(.a(new_n142), .b(new_n146), .o1(new_n147));
  nanb02aa1n06x5               g052(.a(new_n100), .b(new_n102), .out0(new_n148));
  nand03aa1n04x5               g053(.a(new_n99), .b(new_n133), .c(new_n137), .o1(new_n149));
  nona22aa1n06x5               g054(.a(new_n147), .b(new_n148), .c(new_n149), .out0(new_n150));
  nona23aa1n03x5               g055(.a(new_n136), .b(new_n132), .c(new_n131), .d(new_n135), .out0(new_n151));
  tech160nm_fioai012aa1n05x5   g056(.a(new_n98), .b(new_n100), .c(new_n97), .o1(new_n152));
  tech160nm_fiao0012aa1n02p5x5 g057(.a(new_n135), .b(new_n131), .c(new_n136), .o(new_n153));
  oabi12aa1n12x5               g058(.a(new_n153), .b(new_n151), .c(new_n152), .out0(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  tech160nm_fixorc02aa1n05x5   g060(.a(\a[13] ), .b(\b[12] ), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n150), .c(new_n155), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(\a[13] ), .o1(new_n158));
  inv040aa1d32x5               g063(.a(\a[14] ), .o1(new_n159));
  xroi22aa1d06x4               g064(.a(new_n158), .b(\b[12] ), .c(new_n159), .d(\b[13] ), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  inv000aa1d42x5               g066(.a(\b[13] ), .o1(new_n162));
  nor042aa1n03x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  oaoi03aa1n12x5               g068(.a(new_n159), .b(new_n162), .c(new_n163), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n164), .b(new_n161), .c(new_n150), .d(new_n155), .o1(new_n165));
  nand42aa1n03x5               g070(.a(new_n150), .b(new_n155), .o1(new_n166));
  xorc02aa1n02x5               g071(.a(\a[14] ), .b(\b[13] ), .out0(new_n167));
  aoi112aa1n02x5               g072(.a(new_n163), .b(new_n167), .c(new_n166), .d(new_n156), .o1(new_n168));
  oaoi13aa1n03x5               g073(.a(new_n168), .b(new_n165), .c(\a[14] ), .d(\b[13] ), .o1(\s[14] ));
  xorb03aa1n02x5               g074(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  tech160nm_fixnrc02aa1n04x5   g075(.a(\b[14] ), .b(\a[15] ), .out0(new_n171));
  nanb02aa1n03x5               g076(.a(new_n171), .b(new_n165), .out0(new_n172));
  inv000aa1d42x5               g077(.a(\a[16] ), .o1(new_n173));
  inv000aa1d42x5               g078(.a(\b[15] ), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(new_n174), .b(new_n173), .o1(new_n175));
  norp02aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  xorc02aa1n12x5               g081(.a(\a[16] ), .b(\b[15] ), .out0(new_n177));
  norp02aa1n02x5               g082(.a(new_n177), .b(new_n176), .o1(new_n178));
  nanb02aa1n18x5               g083(.a(new_n171), .b(new_n177), .out0(new_n179));
  nanb03aa1n06x5               g084(.a(new_n148), .b(new_n167), .c(new_n156), .out0(new_n180));
  nor043aa1n09x5               g085(.a(new_n180), .b(new_n179), .c(new_n149), .o1(new_n181));
  aoai13aa1n03x5               g086(.a(new_n181), .b(new_n126), .c(new_n112), .d(new_n122), .o1(new_n182));
  inv000aa1n02x5               g087(.a(new_n164), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n179), .o1(new_n184));
  aoai13aa1n04x5               g089(.a(new_n184), .b(new_n183), .c(new_n154), .d(new_n160), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n173), .b(new_n174), .c(new_n176), .o1(new_n186));
  nand23aa1n03x5               g091(.a(new_n182), .b(new_n185), .c(new_n186), .o1(new_n187));
  aoi022aa1n02x7               g092(.a(new_n172), .b(new_n178), .c(new_n175), .d(new_n187), .o1(\s[16] ));
  xorb03aa1n03x5               g093(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor002aa1d32x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nona22aa1n02x4               g095(.a(new_n184), .b(new_n180), .c(new_n149), .out0(new_n191));
  aoi012aa1n02x7               g096(.a(new_n191), .b(new_n142), .c(new_n146), .o1(new_n192));
  nano23aa1n02x4               g097(.a(new_n131), .b(new_n135), .c(new_n136), .d(new_n132), .out0(new_n193));
  inv000aa1n02x5               g098(.a(new_n152), .o1(new_n194));
  aoai13aa1n06x5               g099(.a(new_n160), .b(new_n153), .c(new_n193), .d(new_n194), .o1(new_n195));
  aoai13aa1n12x5               g100(.a(new_n186), .b(new_n179), .c(new_n195), .d(new_n164), .o1(new_n196));
  tech160nm_fixorc02aa1n05x5   g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  nand42aa1n10x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nano22aa1n03x7               g103(.a(new_n190), .b(new_n197), .c(new_n198), .out0(new_n199));
  inv000aa1d42x5               g104(.a(\a[17] ), .o1(new_n200));
  inv000aa1d42x5               g105(.a(\b[16] ), .o1(new_n201));
  aoai13aa1n12x5               g106(.a(new_n198), .b(new_n190), .c(new_n200), .d(new_n201), .o1(new_n202));
  inv040aa1n03x5               g107(.a(new_n202), .o1(new_n203));
  oaoi13aa1n03x5               g108(.a(new_n203), .b(new_n199), .c(new_n192), .d(new_n196), .o1(new_n204));
  obai22aa1n02x7               g109(.a(new_n198), .b(new_n190), .c(\a[17] ), .d(\b[16] ), .out0(new_n205));
  oaoi13aa1n03x5               g110(.a(new_n205), .b(new_n197), .c(new_n192), .d(new_n196), .o1(new_n206));
  oab012aa1n02x5               g111(.a(new_n206), .b(new_n204), .c(new_n190), .out0(\s[18] ));
  aoai13aa1n06x5               g112(.a(new_n199), .b(new_n196), .c(new_n147), .d(new_n181), .o1(new_n208));
  nor002aa1d32x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nand22aa1n04x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nanb02aa1n06x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  xobna2aa1n03x5               g116(.a(new_n211), .b(new_n208), .c(new_n202), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g118(.a(new_n209), .o1(new_n214));
  tech160nm_fiaoi012aa1n03p5x5 g119(.a(new_n211), .b(new_n208), .c(new_n202), .o1(new_n215));
  nor002aa1n03x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nand02aa1n06x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nanb02aa1n09x5               g122(.a(new_n216), .b(new_n217), .out0(new_n218));
  nano22aa1n03x5               g123(.a(new_n215), .b(new_n214), .c(new_n218), .out0(new_n219));
  oaoi13aa1n02x7               g124(.a(new_n218), .b(new_n214), .c(new_n204), .d(new_n211), .o1(new_n220));
  norp02aa1n03x5               g125(.a(new_n220), .b(new_n219), .o1(\s[20] ));
  nano23aa1n06x5               g126(.a(new_n209), .b(new_n216), .c(new_n217), .d(new_n210), .out0(new_n222));
  nanp02aa1n04x5               g127(.a(new_n199), .b(new_n222), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n196), .c(new_n147), .d(new_n181), .o1(new_n225));
  tech160nm_fiaoi012aa1n05x5   g130(.a(new_n216), .b(new_n209), .c(new_n217), .o1(new_n226));
  oai013aa1d12x5               g131(.a(new_n226), .b(new_n202), .c(new_n211), .d(new_n218), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[20] ), .b(\a[21] ), .out0(new_n229));
  xobna2aa1n03x5               g134(.a(new_n229), .b(new_n225), .c(new_n228), .out0(\s[21] ));
  nor002aa1d32x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  tech160nm_fiaoi012aa1n03p5x5 g137(.a(new_n229), .b(new_n225), .c(new_n228), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[21] ), .b(\a[22] ), .out0(new_n234));
  nano22aa1n03x5               g139(.a(new_n233), .b(new_n232), .c(new_n234), .out0(new_n235));
  oaoi13aa1n03x5               g140(.a(new_n227), .b(new_n224), .c(new_n192), .d(new_n196), .o1(new_n236));
  oaoi13aa1n02x7               g141(.a(new_n234), .b(new_n232), .c(new_n236), .d(new_n229), .o1(new_n237));
  norp02aa1n03x5               g142(.a(new_n237), .b(new_n235), .o1(\s[22] ));
  norp02aa1n04x5               g143(.a(new_n234), .b(new_n229), .o1(new_n239));
  and003aa1n02x5               g144(.a(new_n199), .b(new_n239), .c(new_n222), .o(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n196), .c(new_n147), .d(new_n181), .o1(new_n241));
  oao003aa1n12x5               g146(.a(\a[22] ), .b(\b[21] ), .c(new_n232), .carry(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoi012aa1n02x7               g148(.a(new_n243), .b(new_n227), .c(new_n239), .o1(new_n244));
  xnrc02aa1n12x5               g149(.a(\b[22] ), .b(\a[23] ), .out0(new_n245));
  xobna2aa1n03x5               g150(.a(new_n245), .b(new_n241), .c(new_n244), .out0(\s[23] ));
  nor042aa1n06x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  tech160nm_fiaoi012aa1n02p5x5 g153(.a(new_n245), .b(new_n241), .c(new_n244), .o1(new_n249));
  xnrc02aa1n12x5               g154(.a(\b[23] ), .b(\a[24] ), .out0(new_n250));
  nano22aa1n03x5               g155(.a(new_n249), .b(new_n248), .c(new_n250), .out0(new_n251));
  inv000aa1n03x5               g156(.a(new_n244), .o1(new_n252));
  oaoi13aa1n03x5               g157(.a(new_n252), .b(new_n240), .c(new_n192), .d(new_n196), .o1(new_n253));
  oaoi13aa1n02x7               g158(.a(new_n250), .b(new_n248), .c(new_n253), .d(new_n245), .o1(new_n254));
  norp02aa1n03x5               g159(.a(new_n254), .b(new_n251), .o1(\s[24] ));
  nor002aa1n02x5               g160(.a(new_n250), .b(new_n245), .o1(new_n256));
  nano22aa1n02x5               g161(.a(new_n223), .b(new_n239), .c(new_n256), .out0(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n196), .c(new_n147), .d(new_n181), .o1(new_n258));
  inv000aa1n03x5               g163(.a(new_n226), .o1(new_n259));
  aoai13aa1n06x5               g164(.a(new_n239), .b(new_n259), .c(new_n222), .d(new_n203), .o1(new_n260));
  inv000aa1n02x5               g165(.a(new_n256), .o1(new_n261));
  oao003aa1n02x5               g166(.a(\a[24] ), .b(\b[23] ), .c(new_n248), .carry(new_n262));
  aoai13aa1n12x5               g167(.a(new_n262), .b(new_n261), .c(new_n260), .d(new_n242), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  xnrc02aa1n12x5               g169(.a(\b[24] ), .b(\a[25] ), .out0(new_n265));
  xobna2aa1n03x5               g170(.a(new_n265), .b(new_n258), .c(new_n264), .out0(\s[25] ));
  nor042aa1n03x5               g171(.a(\b[24] ), .b(\a[25] ), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  tech160nm_fiaoi012aa1n02p5x5 g173(.a(new_n265), .b(new_n258), .c(new_n264), .o1(new_n269));
  xnrc02aa1n12x5               g174(.a(\b[25] ), .b(\a[26] ), .out0(new_n270));
  nano22aa1n03x5               g175(.a(new_n269), .b(new_n268), .c(new_n270), .out0(new_n271));
  oaoi13aa1n03x5               g176(.a(new_n263), .b(new_n257), .c(new_n192), .d(new_n196), .o1(new_n272));
  oaoi13aa1n02x7               g177(.a(new_n270), .b(new_n268), .c(new_n272), .d(new_n265), .o1(new_n273));
  norp02aa1n03x5               g178(.a(new_n273), .b(new_n271), .o1(\s[26] ));
  nor002aa1n03x5               g179(.a(new_n270), .b(new_n265), .o1(new_n275));
  nano32aa1n06x5               g180(.a(new_n223), .b(new_n275), .c(new_n239), .d(new_n256), .out0(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n196), .c(new_n147), .d(new_n181), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .carry(new_n278));
  aobi12aa1n09x5               g183(.a(new_n278), .b(new_n263), .c(new_n275), .out0(new_n279));
  xorc02aa1n12x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n279), .out0(\s[27] ));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  inv040aa1n03x5               g187(.a(new_n282), .o1(new_n283));
  aobi12aa1n02x7               g188(.a(new_n280), .b(new_n277), .c(new_n279), .out0(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  nano22aa1n03x5               g190(.a(new_n284), .b(new_n283), .c(new_n285), .out0(new_n286));
  aoai13aa1n02x5               g191(.a(new_n256), .b(new_n243), .c(new_n227), .d(new_n239), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n275), .o1(new_n288));
  aoai13aa1n06x5               g193(.a(new_n278), .b(new_n288), .c(new_n287), .d(new_n262), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n280), .b(new_n289), .c(new_n187), .d(new_n276), .o1(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n285), .b(new_n290), .c(new_n283), .o1(new_n291));
  norp02aa1n03x5               g196(.a(new_n291), .b(new_n286), .o1(\s[28] ));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  norb02aa1n02x5               g198(.a(new_n280), .b(new_n285), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n289), .c(new_n187), .d(new_n276), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n296));
  tech160nm_fiaoi012aa1n02p5x5 g201(.a(new_n293), .b(new_n295), .c(new_n296), .o1(new_n297));
  aobi12aa1n02x7               g202(.a(new_n294), .b(new_n277), .c(new_n279), .out0(new_n298));
  nano22aa1n03x5               g203(.a(new_n298), .b(new_n293), .c(new_n296), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n114), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g206(.a(new_n280), .b(new_n293), .c(new_n285), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n289), .c(new_n187), .d(new_n276), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .out0(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  aobi12aa1n02x7               g211(.a(new_n302), .b(new_n277), .c(new_n279), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n307), .b(new_n304), .c(new_n305), .out0(new_n308));
  norp02aa1n03x5               g213(.a(new_n306), .b(new_n308), .o1(\s[30] ));
  norb02aa1n02x5               g214(.a(new_n302), .b(new_n305), .out0(new_n310));
  aobi12aa1n02x7               g215(.a(new_n310), .b(new_n277), .c(new_n279), .out0(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  nano22aa1n03x5               g218(.a(new_n311), .b(new_n312), .c(new_n313), .out0(new_n314));
  aoai13aa1n03x5               g219(.a(new_n310), .b(new_n289), .c(new_n187), .d(new_n276), .o1(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n313), .b(new_n315), .c(new_n312), .o1(new_n316));
  norp02aa1n03x5               g221(.a(new_n316), .b(new_n314), .o1(\s[31] ));
  norb02aa1n02x5               g222(.a(new_n119), .b(new_n118), .out0(new_n318));
  oaoi13aa1n02x5               g223(.a(new_n318), .b(new_n115), .c(new_n114), .d(new_n116), .o1(new_n319));
  oab012aa1n02x4               g224(.a(new_n319), .b(new_n117), .c(new_n120), .out0(\s[3] ));
  oabi12aa1n02x5               g225(.a(new_n118), .b(new_n117), .c(new_n120), .out0(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g227(.a(new_n122), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g228(.a(new_n141), .b(new_n140), .o1(new_n324));
  oao003aa1n03x5               g229(.a(\a[5] ), .b(\b[4] ), .c(new_n324), .carry(new_n325));
  xnbna2aa1n03x5               g230(.a(new_n325), .b(new_n109), .c(new_n110), .out0(\s[6] ));
  tech160nm_fioaoi03aa1n03p5x5 g231(.a(\a[6] ), .b(\b[5] ), .c(new_n325), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n03x5               g233(.a(new_n105), .b(new_n327), .c(new_n106), .o1(new_n329));
  xnbna2aa1n03x5               g234(.a(new_n329), .b(new_n143), .c(new_n104), .out0(\s[8] ));
  xobna2aa1n03x5               g235(.a(new_n148), .b(new_n142), .c(new_n146), .out0(\s[9] ));
endmodule


