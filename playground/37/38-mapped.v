// Benchmark "adder" written by ABC on Thu Jul 18 07:16:02 2024

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
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n321, new_n322, new_n325, new_n326,
    new_n327, new_n329, new_n330, new_n332, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fioai012aa1n04x5   g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nor022aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n03x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1n02x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n105), .b(new_n103), .c(new_n106), .d(new_n104), .out0(new_n107));
  aoi012aa1n02x5               g012(.a(new_n106), .b(new_n104), .c(new_n105), .o1(new_n108));
  oai012aa1n09x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .out0(new_n110));
  nor042aa1n06x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nand42aa1n04x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nanb02aa1n12x5               g017(.a(new_n111), .b(new_n112), .out0(new_n113));
  nor022aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norp02aa1n12x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nona23aa1n02x4               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  nor043aa1n06x5               g023(.a(new_n118), .b(new_n113), .c(new_n110), .o1(new_n119));
  oaoi13aa1n02x7               g024(.a(new_n114), .b(new_n112), .c(new_n116), .d(new_n111), .o1(new_n120));
  aoi022aa1n02x5               g025(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n121));
  obai22aa1n09x5               g026(.a(new_n121), .b(new_n120), .c(\a[8] ), .d(\b[7] ), .out0(new_n122));
  nand42aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n02x5               g028(.a(new_n123), .b(new_n97), .out0(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n122), .c(new_n109), .d(new_n119), .o1(new_n125));
  nor042aa1n06x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand42aa1n16x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g034(.a(new_n126), .o1(new_n130));
  inv000aa1d42x5               g035(.a(new_n127), .o1(new_n131));
  aoi013aa1n02x4               g036(.a(new_n131), .b(new_n125), .c(new_n130), .d(new_n98), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n16x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  aoi113aa1n03x7               g041(.a(new_n136), .b(new_n131), .c(new_n125), .d(new_n130), .e(new_n98), .o1(new_n137));
  nor042aa1n09x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1n04x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  oab012aa1n02x4               g045(.a(new_n140), .b(new_n137), .c(new_n134), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n138), .o1(new_n142));
  aoi112aa1n02x5               g047(.a(new_n137), .b(new_n134), .c(new_n142), .d(new_n139), .o1(new_n143));
  norp02aa1n02x5               g048(.a(new_n141), .b(new_n143), .o1(\s[12] ));
  nano23aa1n06x5               g049(.a(new_n134), .b(new_n138), .c(new_n139), .d(new_n135), .out0(new_n145));
  nano23aa1n06x5               g050(.a(new_n97), .b(new_n126), .c(new_n127), .d(new_n123), .out0(new_n146));
  nand22aa1n09x5               g051(.a(new_n146), .b(new_n145), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n122), .c(new_n109), .d(new_n119), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n134), .b(new_n139), .o1(new_n150));
  tech160nm_fioai012aa1n04x5   g055(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n151));
  norp03aa1n06x5               g056(.a(new_n151), .b(new_n136), .c(new_n140), .o1(new_n152));
  nano22aa1n02x5               g057(.a(new_n152), .b(new_n142), .c(new_n150), .out0(new_n153));
  nor042aa1n09x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand42aa1n03x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n03x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n149), .c(new_n153), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(new_n154), .o1(new_n158));
  nand42aa1n02x5               g063(.a(new_n109), .b(new_n119), .o1(new_n159));
  norp02aa1n02x5               g064(.a(\b[7] ), .b(\a[8] ), .o1(new_n160));
  oa0012aa1n09x5               g065(.a(new_n112), .b(new_n116), .c(new_n111), .o(new_n161));
  oaoi13aa1n02x7               g066(.a(new_n160), .b(new_n121), .c(new_n161), .d(new_n114), .o1(new_n162));
  nanp02aa1n03x5               g067(.a(new_n159), .b(new_n162), .o1(new_n163));
  nona23aa1n02x4               g068(.a(new_n139), .b(new_n135), .c(new_n134), .d(new_n138), .out0(new_n164));
  oai112aa1n06x5               g069(.a(new_n150), .b(new_n142), .c(new_n164), .d(new_n151), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n156), .b(new_n165), .c(new_n163), .d(new_n148), .o1(new_n166));
  nor042aa1n02x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand02aa1n03x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  norb02aa1n03x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n166), .c(new_n158), .out0(\s[14] ));
  nona23aa1n09x5               g075(.a(new_n168), .b(new_n155), .c(new_n154), .d(new_n167), .out0(new_n171));
  tech160nm_fioai012aa1n04x5   g076(.a(new_n168), .b(new_n167), .c(new_n154), .o1(new_n172));
  aoai13aa1n06x5               g077(.a(new_n172), .b(new_n171), .c(new_n149), .d(new_n153), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n04x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nand42aa1n03x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nor002aa1n03x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand42aa1n03x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  aoai13aa1n02x7               g084(.a(new_n179), .b(new_n175), .c(new_n173), .d(new_n176), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n175), .b(new_n179), .c(new_n173), .d(new_n176), .o1(new_n181));
  norb02aa1n03x4               g086(.a(new_n180), .b(new_n181), .out0(\s[16] ));
  nano23aa1n03x7               g087(.a(new_n175), .b(new_n177), .c(new_n178), .d(new_n176), .out0(new_n183));
  nano32aa1n03x7               g088(.a(new_n147), .b(new_n183), .c(new_n156), .d(new_n169), .out0(new_n184));
  aoai13aa1n12x5               g089(.a(new_n184), .b(new_n122), .c(new_n109), .d(new_n119), .o1(new_n185));
  nona23aa1n09x5               g090(.a(new_n178), .b(new_n176), .c(new_n175), .d(new_n177), .out0(new_n186));
  nor042aa1n06x5               g091(.a(new_n186), .b(new_n171), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(new_n175), .b(new_n178), .o1(new_n188));
  oai122aa1n06x5               g093(.a(new_n188), .b(new_n186), .c(new_n172), .d(\b[15] ), .e(\a[16] ), .o1(new_n189));
  aoi012aa1d18x5               g094(.a(new_n189), .b(new_n165), .c(new_n187), .o1(new_n190));
  xorc02aa1n02x5               g095(.a(\a[17] ), .b(\b[16] ), .out0(new_n191));
  xnbna2aa1n03x5               g096(.a(new_n191), .b(new_n185), .c(new_n190), .out0(\s[17] ));
  inv040aa1d32x5               g097(.a(\a[18] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(new_n185), .b(new_n190), .o1(new_n194));
  norp02aa1n02x5               g099(.a(\b[16] ), .b(\a[17] ), .o1(new_n195));
  aoi012aa1n03x5               g100(.a(new_n195), .b(new_n194), .c(new_n191), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n193), .out0(\s[18] ));
  inv000aa1d42x5               g102(.a(\a[17] ), .o1(new_n198));
  xroi22aa1d06x4               g103(.a(new_n198), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  oaih22aa1n04x5               g106(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n202));
  and002aa1n02x5               g107(.a(new_n202), .b(new_n201), .o(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n204), .b(new_n200), .c(new_n185), .d(new_n190), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nanp02aa1n04x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nor042aa1n04x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nand42aa1n04x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  aoai13aa1n03x5               g117(.a(new_n212), .b(new_n208), .c(new_n205), .d(new_n209), .o1(new_n213));
  aoi112aa1n02x5               g118(.a(new_n208), .b(new_n212), .c(new_n205), .d(new_n209), .o1(new_n214));
  norb02aa1n03x4               g119(.a(new_n213), .b(new_n214), .out0(\s[20] ));
  nanb03aa1n06x5               g120(.a(new_n210), .b(new_n211), .c(new_n209), .out0(new_n216));
  nona22aa1n03x5               g121(.a(new_n199), .b(new_n208), .c(new_n216), .out0(new_n217));
  oai112aa1n06x5               g122(.a(new_n202), .b(new_n201), .c(\b[18] ), .d(\a[19] ), .o1(new_n218));
  tech160nm_fiaoi012aa1n03p5x5 g123(.a(new_n210), .b(new_n208), .c(new_n211), .o1(new_n219));
  oai012aa1n09x5               g124(.a(new_n219), .b(new_n218), .c(new_n216), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n217), .c(new_n185), .d(new_n190), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  tech160nm_fixnrc02aa1n05x5   g129(.a(\b[20] ), .b(\a[21] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  tech160nm_fixnrc02aa1n05x5   g131(.a(\b[21] ), .b(\a[22] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoai13aa1n03x5               g133(.a(new_n228), .b(new_n224), .c(new_n222), .d(new_n226), .o1(new_n229));
  aoi112aa1n02x5               g134(.a(new_n224), .b(new_n228), .c(new_n222), .d(new_n226), .o1(new_n230));
  norb02aa1n02x7               g135(.a(new_n229), .b(new_n230), .out0(\s[22] ));
  nor042aa1n06x5               g136(.a(new_n227), .b(new_n225), .o1(new_n232));
  nona23aa1n08x5               g137(.a(new_n199), .b(new_n232), .c(new_n216), .d(new_n208), .out0(new_n233));
  nano22aa1n02x4               g138(.a(new_n210), .b(new_n209), .c(new_n211), .out0(new_n234));
  oai012aa1n02x5               g139(.a(new_n201), .b(\b[18] ), .c(\a[19] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n202), .b(new_n235), .out0(new_n236));
  inv040aa1n02x5               g141(.a(new_n219), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n232), .b(new_n237), .c(new_n236), .d(new_n234), .o1(new_n238));
  inv000aa1d42x5               g143(.a(\a[22] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(\b[21] ), .o1(new_n240));
  oao003aa1n12x5               g145(.a(new_n239), .b(new_n240), .c(new_n224), .carry(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  nand02aa1n02x5               g147(.a(new_n238), .b(new_n242), .o1(new_n243));
  inv000aa1n02x5               g148(.a(new_n243), .o1(new_n244));
  aoai13aa1n04x5               g149(.a(new_n244), .b(new_n233), .c(new_n185), .d(new_n190), .o1(new_n245));
  xorb03aa1n02x5               g150(.a(new_n245), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  tech160nm_fixorc02aa1n02p5x5 g152(.a(\a[23] ), .b(\b[22] ), .out0(new_n248));
  xorc02aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  aoai13aa1n03x5               g154(.a(new_n249), .b(new_n247), .c(new_n245), .d(new_n248), .o1(new_n250));
  aoi112aa1n02x7               g155(.a(new_n247), .b(new_n249), .c(new_n245), .d(new_n248), .o1(new_n251));
  norb02aa1n02x7               g156(.a(new_n250), .b(new_n251), .out0(\s[24] ));
  nano32aa1n03x7               g157(.a(new_n217), .b(new_n249), .c(new_n232), .d(new_n248), .out0(new_n253));
  inv000aa1n02x5               g158(.a(new_n253), .o1(new_n254));
  and002aa1n06x5               g159(.a(new_n249), .b(new_n248), .o(new_n255));
  inv000aa1d42x5               g160(.a(\a[24] ), .o1(new_n256));
  inv000aa1d42x5               g161(.a(\b[23] ), .o1(new_n257));
  oao003aa1n02x5               g162(.a(new_n256), .b(new_n257), .c(new_n247), .carry(new_n258));
  tech160nm_fiaoi012aa1n05x5   g163(.a(new_n258), .b(new_n243), .c(new_n255), .o1(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n254), .c(new_n185), .d(new_n190), .o1(new_n260));
  xorb03aa1n02x5               g165(.a(new_n260), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .o1(new_n262));
  xorc02aa1n03x5               g167(.a(\a[25] ), .b(\b[24] ), .out0(new_n263));
  xorc02aa1n02x5               g168(.a(\a[26] ), .b(\b[25] ), .out0(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n262), .c(new_n260), .d(new_n263), .o1(new_n265));
  aoi112aa1n02x5               g170(.a(new_n262), .b(new_n264), .c(new_n260), .d(new_n263), .o1(new_n266));
  norb02aa1n02x7               g171(.a(new_n265), .b(new_n266), .out0(\s[26] ));
  inv000aa1n02x5               g172(.a(new_n187), .o1(new_n268));
  oabi12aa1n03x5               g173(.a(new_n189), .b(new_n153), .c(new_n268), .out0(new_n269));
  and002aa1n06x5               g174(.a(new_n264), .b(new_n263), .o(new_n270));
  nano22aa1n03x7               g175(.a(new_n233), .b(new_n255), .c(new_n270), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n269), .c(new_n163), .d(new_n184), .o1(new_n272));
  inv000aa1n02x5               g177(.a(new_n255), .o1(new_n273));
  inv030aa1n02x5               g178(.a(new_n258), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n273), .c(new_n238), .d(new_n242), .o1(new_n275));
  oai022aa1n02x5               g180(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n276));
  aob012aa1n02x5               g181(.a(new_n276), .b(\b[25] ), .c(\a[26] ), .out0(new_n277));
  aobi12aa1n06x5               g182(.a(new_n277), .b(new_n275), .c(new_n270), .out0(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n278), .c(new_n272), .out0(\s[27] ));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  inv040aa1n03x5               g186(.a(new_n281), .o1(new_n282));
  nand02aa1n02x5               g187(.a(new_n253), .b(new_n270), .o1(new_n283));
  aoi012aa1n06x5               g188(.a(new_n283), .b(new_n185), .c(new_n190), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n255), .b(new_n241), .c(new_n220), .d(new_n232), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n270), .o1(new_n286));
  aoai13aa1n06x5               g191(.a(new_n277), .b(new_n286), .c(new_n285), .d(new_n274), .o1(new_n287));
  oaih12aa1n02x5               g192(.a(new_n279), .b(new_n287), .c(new_n284), .o1(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[27] ), .b(\a[28] ), .out0(new_n289));
  tech160nm_fiaoi012aa1n02p5x5 g194(.a(new_n289), .b(new_n288), .c(new_n282), .o1(new_n290));
  aobi12aa1n02x7               g195(.a(new_n279), .b(new_n278), .c(new_n272), .out0(new_n291));
  nano22aa1n03x5               g196(.a(new_n291), .b(new_n282), .c(new_n289), .out0(new_n292));
  norp02aa1n03x5               g197(.a(new_n290), .b(new_n292), .o1(\s[28] ));
  norb02aa1n02x5               g198(.a(new_n279), .b(new_n289), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n287), .c(new_n284), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x7               g203(.a(new_n294), .b(new_n278), .c(new_n272), .out0(new_n299));
  nano22aa1n03x5               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g207(.a(new_n279), .b(new_n297), .c(new_n289), .out0(new_n303));
  oaih12aa1n02x5               g208(.a(new_n303), .b(new_n287), .c(new_n284), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n02x7               g212(.a(new_n303), .b(new_n278), .c(new_n272), .out0(new_n308));
  nano22aa1n03x5               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[30] ));
  norb02aa1n02x5               g215(.a(new_n303), .b(new_n306), .out0(new_n311));
  aobi12aa1n02x7               g216(.a(new_n311), .b(new_n278), .c(new_n272), .out0(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[30] ), .b(\a[31] ), .out0(new_n314));
  nano22aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n314), .out0(new_n315));
  oaih12aa1n02x5               g220(.a(new_n311), .b(new_n287), .c(new_n284), .o1(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n314), .b(new_n316), .c(new_n313), .o1(new_n317));
  norp02aa1n03x5               g222(.a(new_n317), .b(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g223(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  norb02aa1n02x5               g224(.a(new_n105), .b(new_n106), .out0(new_n320));
  oaib12aa1n02x5               g225(.a(new_n103), .b(new_n104), .c(new_n102), .out0(new_n321));
  oai022aa1n02x5               g226(.a(new_n107), .b(new_n102), .c(\b[2] ), .d(\a[3] ), .o1(new_n322));
  mtn022aa1n02x5               g227(.a(new_n322), .b(new_n321), .sa(new_n320), .o1(\s[4] ));
  xorb03aa1n02x5               g228(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g229(.a(new_n113), .o1(new_n325));
  aoai13aa1n02x5               g230(.a(new_n325), .b(new_n116), .c(new_n109), .d(new_n117), .o1(new_n326));
  aoi112aa1n02x5               g231(.a(new_n116), .b(new_n325), .c(new_n109), .d(new_n117), .o1(new_n327));
  norb02aa1n02x5               g232(.a(new_n326), .b(new_n327), .out0(\s[6] ));
  norb02aa1n02x5               g233(.a(new_n115), .b(new_n114), .out0(new_n329));
  inv000aa1d42x5               g234(.a(new_n161), .o1(new_n330));
  xnbna2aa1n03x5               g235(.a(new_n329), .b(new_n326), .c(new_n330), .out0(\s[7] ));
  nanp02aa1n02x5               g236(.a(new_n326), .b(new_n330), .o1(new_n332));
  aoi012aa1n02x5               g237(.a(new_n114), .b(new_n332), .c(new_n115), .o1(new_n333));
  xnrb03aa1n02x5               g238(.a(new_n333), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g239(.a(new_n124), .b(new_n159), .c(new_n162), .out0(\s[9] ));
endmodule


