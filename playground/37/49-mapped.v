// Benchmark "adder" written by ABC on Thu Jul 18 07:22:49 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n324,
    new_n325, new_n326, new_n329, new_n331, new_n332, new_n333, new_n334,
    new_n336;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1n03x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor002aa1n04x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  norp02aa1n04x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  oa0012aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o(new_n102));
  nanp02aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand22aa1n03x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  nor042aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  oai012aa1n12x5               g010(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n03x5               g012(.a(new_n99), .b(new_n107), .c(new_n101), .d(new_n100), .out0(new_n108));
  oabi12aa1n06x5               g013(.a(new_n102), .b(new_n106), .c(new_n108), .out0(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .out0(new_n110));
  norp02aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nor042aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nona23aa1n02x4               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  nor022aa1n08x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nand42aa1n04x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanb02aa1n09x5               g022(.a(new_n116), .b(new_n117), .out0(new_n118));
  nor043aa1n02x5               g023(.a(new_n115), .b(new_n118), .c(new_n110), .o1(new_n119));
  oaoi13aa1n06x5               g024(.a(new_n111), .b(new_n117), .c(new_n116), .d(new_n113), .o1(new_n120));
  aoi022aa1n02x5               g025(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n121));
  obai22aa1n02x7               g026(.a(new_n121), .b(new_n120), .c(\a[8] ), .d(\b[7] ), .out0(new_n122));
  nanp02aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n02x5               g028(.a(new_n123), .b(new_n97), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n122), .c(new_n109), .d(new_n119), .o1(new_n125));
  nor002aa1n16x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand02aa1n04x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1d27x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g034(.a(new_n128), .o1(new_n130));
  oai012aa1n12x5               g035(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n131));
  aoai13aa1n02x5               g036(.a(new_n131), .b(new_n130), .c(new_n125), .d(new_n98), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n16x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand02aa1d06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor002aa1d32x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand02aa1d06x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoi112aa1n02x5               g043(.a(new_n134), .b(new_n138), .c(new_n132), .d(new_n135), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n135), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(\s[12] ));
  inv000aa1n02x5               g046(.a(new_n106), .o1(new_n142));
  nano23aa1n02x4               g047(.a(new_n101), .b(new_n100), .c(new_n107), .d(new_n99), .out0(new_n143));
  aoi012aa1n02x5               g048(.a(new_n102), .b(new_n143), .c(new_n142), .o1(new_n144));
  nano23aa1n02x4               g049(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n145));
  nona22aa1n02x4               g050(.a(new_n145), .b(new_n110), .c(new_n118), .out0(new_n146));
  norp02aa1n02x5               g051(.a(\b[7] ), .b(\a[8] ), .o1(new_n147));
  oa0012aa1n02x5               g052(.a(new_n117), .b(new_n116), .c(new_n113), .o(new_n148));
  oaoi13aa1n09x5               g053(.a(new_n147), .b(new_n121), .c(new_n148), .d(new_n111), .o1(new_n149));
  oai012aa1n02x5               g054(.a(new_n149), .b(new_n144), .c(new_n146), .o1(new_n150));
  nona23aa1d18x5               g055(.a(new_n137), .b(new_n135), .c(new_n134), .d(new_n136), .out0(new_n151));
  nano22aa1n02x4               g056(.a(new_n151), .b(new_n124), .c(new_n128), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n136), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n134), .b(new_n137), .o1(new_n154));
  oai112aa1n06x5               g059(.a(new_n154), .b(new_n153), .c(new_n151), .d(new_n131), .o1(new_n155));
  aoi012aa1n03x5               g060(.a(new_n155), .b(new_n150), .c(new_n152), .o1(new_n156));
  xnrb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  oaoi03aa1n03x5               g062(.a(\a[13] ), .b(\b[12] ), .c(new_n156), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  aoai13aa1n02x5               g064(.a(new_n152), .b(new_n122), .c(new_n109), .d(new_n119), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n155), .o1(new_n161));
  norp02aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand42aa1n02x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  norp02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand42aa1n03x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nona23aa1n02x4               g070(.a(new_n165), .b(new_n163), .c(new_n162), .d(new_n164), .out0(new_n166));
  oai012aa1n02x5               g071(.a(new_n165), .b(new_n164), .c(new_n162), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n166), .c(new_n160), .d(new_n161), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  norb02aa1n03x4               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  norp02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n03x4               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  aoi112aa1n02x5               g080(.a(new_n170), .b(new_n175), .c(new_n168), .d(new_n172), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n175), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(\s[16] ));
  nano23aa1n03x7               g083(.a(new_n134), .b(new_n136), .c(new_n137), .d(new_n135), .out0(new_n179));
  nano23aa1n02x5               g084(.a(new_n162), .b(new_n164), .c(new_n165), .d(new_n163), .out0(new_n180));
  nanp03aa1n02x5               g085(.a(new_n180), .b(new_n172), .c(new_n175), .o1(new_n181));
  nano32aa1n02x4               g086(.a(new_n181), .b(new_n179), .c(new_n128), .d(new_n124), .out0(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n122), .c(new_n109), .d(new_n119), .o1(new_n183));
  nona23aa1n02x4               g088(.a(new_n174), .b(new_n171), .c(new_n170), .d(new_n173), .out0(new_n184));
  nor042aa1n02x5               g089(.a(new_n184), .b(new_n166), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n186));
  oai022aa1n02x5               g091(.a(new_n184), .b(new_n167), .c(\b[15] ), .d(\a[16] ), .o1(new_n187));
  aoi112aa1n09x5               g092(.a(new_n187), .b(new_n186), .c(new_n155), .d(new_n185), .o1(new_n188));
  norp02aa1n04x5               g093(.a(\b[16] ), .b(\a[17] ), .o1(new_n189));
  nanp02aa1n04x5               g094(.a(\b[16] ), .b(\a[17] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  xnbna2aa1n03x5               g096(.a(new_n191), .b(new_n188), .c(new_n183), .out0(\s[17] ));
  nanp02aa1n02x5               g097(.a(new_n152), .b(new_n185), .o1(new_n193));
  oaoi13aa1n06x5               g098(.a(new_n193), .b(new_n149), .c(new_n144), .d(new_n146), .o1(new_n194));
  nand42aa1n03x5               g099(.a(new_n155), .b(new_n185), .o1(new_n195));
  nona22aa1n02x5               g100(.a(new_n195), .b(new_n187), .c(new_n186), .out0(new_n196));
  oaoi13aa1n03x5               g101(.a(new_n189), .b(new_n191), .c(new_n196), .d(new_n194), .o1(new_n197));
  xnrb03aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor002aa1n02x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nand42aa1n03x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nano23aa1n06x5               g105(.a(new_n189), .b(new_n199), .c(new_n200), .d(new_n190), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  oa0012aa1n02x5               g107(.a(new_n200), .b(new_n199), .c(new_n189), .o(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n204), .b(new_n202), .c(new_n188), .d(new_n183), .o1(new_n205));
  xorb03aa1n03x5               g110(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n03x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nand42aa1n02x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  norp02aa1n04x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanp02aa1n04x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  aoi112aa1n02x7               g117(.a(new_n208), .b(new_n212), .c(new_n205), .d(new_n209), .o1(new_n213));
  inv000aa1d42x5               g118(.a(\b[18] ), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(\a[19] ), .b(new_n214), .out0(new_n215));
  norb02aa1n02x5               g120(.a(new_n209), .b(new_n208), .out0(new_n216));
  nand42aa1n02x5               g121(.a(new_n205), .b(new_n216), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n212), .o1(new_n218));
  aoi012aa1n03x5               g123(.a(new_n218), .b(new_n217), .c(new_n215), .o1(new_n219));
  norp02aa1n03x5               g124(.a(new_n219), .b(new_n213), .o1(\s[20] ));
  nano22aa1n02x4               g125(.a(new_n210), .b(new_n209), .c(new_n211), .out0(new_n221));
  nand23aa1n03x5               g126(.a(new_n201), .b(new_n215), .c(new_n221), .o1(new_n222));
  nanb03aa1n06x5               g127(.a(new_n210), .b(new_n211), .c(new_n209), .out0(new_n223));
  oai022aa1n02x5               g128(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n224));
  nanp03aa1n02x5               g129(.a(new_n224), .b(new_n215), .c(new_n200), .o1(new_n225));
  aoi012aa1d18x5               g130(.a(new_n210), .b(new_n208), .c(new_n211), .o1(new_n226));
  oai012aa1n18x5               g131(.a(new_n226), .b(new_n225), .c(new_n223), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n222), .c(new_n188), .d(new_n183), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[20] ), .b(\a[21] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[21] ), .b(\a[22] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoi112aa1n02x7               g140(.a(new_n231), .b(new_n235), .c(new_n229), .d(new_n233), .o1(new_n236));
  inv000aa1n02x5               g141(.a(new_n231), .o1(new_n237));
  tech160nm_finand02aa1n03p5x5 g142(.a(new_n229), .b(new_n233), .o1(new_n238));
  aoi012aa1n03x5               g143(.a(new_n234), .b(new_n238), .c(new_n237), .o1(new_n239));
  norp02aa1n03x5               g144(.a(new_n239), .b(new_n236), .o1(\s[22] ));
  norp02aa1n06x5               g145(.a(new_n234), .b(new_n232), .o1(new_n241));
  nona23aa1n08x5               g146(.a(new_n241), .b(new_n201), .c(new_n208), .d(new_n223), .out0(new_n242));
  oao003aa1n02x5               g147(.a(\a[22] ), .b(\b[21] ), .c(new_n237), .carry(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  aoi012aa1n02x5               g149(.a(new_n244), .b(new_n227), .c(new_n241), .o1(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n242), .c(new_n188), .d(new_n183), .o1(new_n246));
  xorb03aa1n03x5               g151(.a(new_n246), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  tech160nm_fixorc02aa1n05x5   g153(.a(\a[23] ), .b(\b[22] ), .out0(new_n249));
  xorc02aa1n12x5               g154(.a(\a[24] ), .b(\b[23] ), .out0(new_n250));
  aoi112aa1n03x5               g155(.a(new_n248), .b(new_n250), .c(new_n246), .d(new_n249), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n248), .o1(new_n252));
  nand42aa1n04x5               g157(.a(new_n246), .b(new_n249), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n250), .o1(new_n254));
  aoi012aa1n03x5               g159(.a(new_n254), .b(new_n253), .c(new_n252), .o1(new_n255));
  norp02aa1n03x5               g160(.a(new_n255), .b(new_n251), .o1(\s[24] ));
  nano32aa1n03x7               g161(.a(new_n222), .b(new_n250), .c(new_n241), .d(new_n249), .out0(new_n257));
  inv000aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  oai012aa1n02x5               g163(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .o1(new_n259));
  oab012aa1n02x4               g164(.a(new_n259), .b(new_n189), .c(new_n199), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n226), .o1(new_n261));
  aoai13aa1n03x5               g166(.a(new_n241), .b(new_n261), .c(new_n260), .d(new_n221), .o1(new_n262));
  and002aa1n02x5               g167(.a(new_n250), .b(new_n249), .o(new_n263));
  inv000aa1n02x5               g168(.a(new_n263), .o1(new_n264));
  oao003aa1n02x5               g169(.a(\a[24] ), .b(\b[23] ), .c(new_n252), .carry(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n264), .c(new_n262), .d(new_n243), .o1(new_n266));
  inv040aa1n03x5               g171(.a(new_n266), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n258), .c(new_n188), .d(new_n183), .o1(new_n268));
  xorb03aa1n03x5               g173(.a(new_n268), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  tech160nm_fixorc02aa1n03p5x5 g175(.a(\a[25] ), .b(\b[24] ), .out0(new_n271));
  xorc02aa1n12x5               g176(.a(\a[26] ), .b(\b[25] ), .out0(new_n272));
  aoi112aa1n03x5               g177(.a(new_n270), .b(new_n272), .c(new_n268), .d(new_n271), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n270), .o1(new_n274));
  nand42aa1n04x5               g179(.a(new_n268), .b(new_n271), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n272), .o1(new_n276));
  aoi012aa1n03x5               g181(.a(new_n276), .b(new_n275), .c(new_n274), .o1(new_n277));
  norp02aa1n03x5               g182(.a(new_n277), .b(new_n273), .o1(\s[26] ));
  and002aa1n06x5               g183(.a(new_n272), .b(new_n271), .o(new_n279));
  nano22aa1n03x7               g184(.a(new_n242), .b(new_n263), .c(new_n279), .out0(new_n280));
  tech160nm_fioai012aa1n05x5   g185(.a(new_n280), .b(new_n196), .c(new_n194), .o1(new_n281));
  oao003aa1n02x5               g186(.a(\a[26] ), .b(\b[25] ), .c(new_n274), .carry(new_n282));
  aobi12aa1n09x5               g187(.a(new_n282), .b(new_n266), .c(new_n279), .out0(new_n283));
  xorc02aa1n02x5               g188(.a(\a[27] ), .b(\b[26] ), .out0(new_n284));
  xnbna2aa1n03x5               g189(.a(new_n284), .b(new_n281), .c(new_n283), .out0(\s[27] ));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  inv040aa1n03x5               g191(.a(new_n286), .o1(new_n287));
  aobi12aa1n02x5               g192(.a(new_n284), .b(new_n281), .c(new_n283), .out0(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[27] ), .b(\a[28] ), .out0(new_n289));
  nano22aa1n02x4               g194(.a(new_n288), .b(new_n287), .c(new_n289), .out0(new_n290));
  nand02aa1d04x5               g195(.a(new_n188), .b(new_n183), .o1(new_n291));
  aoai13aa1n06x5               g196(.a(new_n263), .b(new_n244), .c(new_n227), .d(new_n241), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n279), .o1(new_n293));
  aoai13aa1n06x5               g198(.a(new_n282), .b(new_n293), .c(new_n292), .d(new_n265), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n284), .b(new_n294), .c(new_n291), .d(new_n280), .o1(new_n295));
  tech160nm_fiaoi012aa1n02p5x5 g200(.a(new_n289), .b(new_n295), .c(new_n287), .o1(new_n296));
  nor002aa1n02x5               g201(.a(new_n296), .b(new_n290), .o1(\s[28] ));
  norb02aa1n02x5               g202(.a(new_n284), .b(new_n289), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n294), .c(new_n291), .d(new_n280), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[28] ), .b(\b[27] ), .c(new_n287), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[28] ), .b(\a[29] ), .out0(new_n301));
  aoi012aa1n03x5               g206(.a(new_n301), .b(new_n299), .c(new_n300), .o1(new_n302));
  aobi12aa1n02x7               g207(.a(new_n298), .b(new_n281), .c(new_n283), .out0(new_n303));
  nano22aa1n03x5               g208(.a(new_n303), .b(new_n300), .c(new_n301), .out0(new_n304));
  norp02aa1n03x5               g209(.a(new_n302), .b(new_n304), .o1(\s[29] ));
  xorb03aa1n02x5               g210(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g211(.a(new_n284), .b(new_n301), .c(new_n289), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n294), .c(new_n291), .d(new_n280), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .carry(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[29] ), .b(\a[30] ), .out0(new_n310));
  tech160nm_fiaoi012aa1n02p5x5 g215(.a(new_n310), .b(new_n308), .c(new_n309), .o1(new_n311));
  aobi12aa1n02x7               g216(.a(new_n307), .b(new_n281), .c(new_n283), .out0(new_n312));
  nano22aa1n02x4               g217(.a(new_n312), .b(new_n309), .c(new_n310), .out0(new_n313));
  norp02aa1n03x5               g218(.a(new_n311), .b(new_n313), .o1(\s[30] ));
  xnrc02aa1n02x5               g219(.a(\b[30] ), .b(\a[31] ), .out0(new_n315));
  norb02aa1n02x5               g220(.a(new_n307), .b(new_n310), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n294), .c(new_n291), .d(new_n280), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .c(new_n309), .carry(new_n318));
  aoi012aa1n03x5               g223(.a(new_n315), .b(new_n317), .c(new_n318), .o1(new_n319));
  aobi12aa1n02x5               g224(.a(new_n316), .b(new_n281), .c(new_n283), .out0(new_n320));
  nano22aa1n03x5               g225(.a(new_n320), .b(new_n315), .c(new_n318), .out0(new_n321));
  norp02aa1n03x5               g226(.a(new_n319), .b(new_n321), .o1(\s[31] ));
  xnrb03aa1n02x5               g227(.a(new_n106), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanb02aa1n02x5               g228(.a(new_n101), .b(new_n99), .out0(new_n324));
  norp02aa1n02x5               g229(.a(new_n108), .b(new_n106), .o1(new_n325));
  oai112aa1n02x5               g230(.a(new_n107), .b(new_n324), .c(new_n142), .d(new_n100), .o1(new_n326));
  oai013aa1n02x4               g231(.a(new_n326), .b(new_n325), .c(new_n100), .d(new_n324), .o1(\s[4] ));
  xorb03aa1n02x5               g232(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g233(.a(\a[5] ), .b(\b[4] ), .c(new_n144), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g235(.a(new_n112), .b(new_n111), .out0(new_n331));
  inv000aa1d42x5               g236(.a(new_n118), .o1(new_n332));
  aoai13aa1n02x5               g237(.a(new_n331), .b(new_n148), .c(new_n329), .d(new_n332), .o1(new_n333));
  aoi112aa1n02x5               g238(.a(new_n148), .b(new_n331), .c(new_n329), .d(new_n332), .o1(new_n334));
  norb02aa1n02x5               g239(.a(new_n333), .b(new_n334), .out0(\s[7] ));
  orn002aa1n02x5               g240(.a(\a[7] ), .b(\b[6] ), .o(new_n336));
  xobna2aa1n03x5               g241(.a(new_n110), .b(new_n333), .c(new_n336), .out0(\s[8] ));
  xorb03aa1n02x5               g242(.a(new_n150), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


