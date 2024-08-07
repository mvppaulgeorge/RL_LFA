// Benchmark "adder" written by ABC on Thu Jul 18 06:34:48 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n323, new_n326,
    new_n328, new_n330, new_n331;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nor042aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d06x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand02aa1d06x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n12x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  xorc02aa1n03x5               g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  nor042aa1d18x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  tech160nm_finand02aa1n03p5x5 g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n03x4               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nanb03aa1n02x5               g011(.a(new_n102), .b(new_n103), .c(new_n106), .out0(new_n107));
  inv040aa1n06x5               g012(.a(new_n104), .o1(new_n108));
  oao003aa1n06x5               g013(.a(\a[4] ), .b(\b[3] ), .c(new_n108), .carry(new_n109));
  nor022aa1n12x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand42aa1n06x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor042aa1n06x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n03x7               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  nand22aa1n12x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor002aa1n08x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nor042aa1n12x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nand42aa1n03x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nona23aa1n09x5               g023(.a(new_n115), .b(new_n118), .c(new_n117), .d(new_n116), .out0(new_n119));
  nanb02aa1n03x5               g024(.a(new_n119), .b(new_n114), .out0(new_n120));
  aoi012aa1d18x5               g025(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n121));
  inv000aa1d42x5               g026(.a(new_n121), .o1(new_n122));
  oai012aa1n02x5               g027(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n123));
  aobi12aa1n06x5               g028(.a(new_n123), .b(new_n114), .c(new_n122), .out0(new_n124));
  aoai13aa1n04x5               g029(.a(new_n124), .b(new_n120), .c(new_n107), .d(new_n109), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(new_n97), .b(new_n98), .c(new_n125), .o1(new_n126));
  xnrb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nand42aa1n03x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nor042aa1n06x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand42aa1d28x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nano23aa1n06x5               g036(.a(new_n128), .b(new_n130), .c(new_n131), .d(new_n129), .out0(new_n132));
  aoai13aa1n12x5               g037(.a(new_n131), .b(new_n130), .c(new_n97), .d(new_n98), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nor042aa1d18x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1d28x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n03x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoai13aa1n03x5               g042(.a(new_n137), .b(new_n134), .c(new_n125), .d(new_n132), .o1(new_n138));
  aoi112aa1n02x5               g043(.a(new_n137), .b(new_n134), .c(new_n125), .d(new_n132), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(\s[11] ));
  inv000aa1d42x5               g045(.a(new_n135), .o1(new_n141));
  xorc02aa1n02x5               g046(.a(\a[12] ), .b(\b[11] ), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n138), .c(new_n141), .out0(\s[12] ));
  tech160nm_fixnrc02aa1n02p5x5 g048(.a(\b[3] ), .b(\a[4] ), .out0(new_n144));
  nanb02aa1n06x5               g049(.a(new_n104), .b(new_n105), .out0(new_n145));
  oai013aa1n09x5               g050(.a(new_n109), .b(new_n144), .c(new_n102), .d(new_n145), .o1(new_n146));
  nanb02aa1n12x5               g051(.a(new_n110), .b(new_n111), .out0(new_n147));
  nanb02aa1n09x5               g052(.a(new_n112), .b(new_n113), .out0(new_n148));
  nor043aa1n03x5               g053(.a(new_n119), .b(new_n148), .c(new_n147), .o1(new_n149));
  oai013aa1n09x5               g054(.a(new_n123), .b(new_n121), .c(new_n147), .d(new_n148), .o1(new_n150));
  orn002aa1n02x5               g055(.a(\a[12] ), .b(\b[11] ), .o(new_n151));
  nanp02aa1n02x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nand23aa1n02x5               g057(.a(new_n137), .b(new_n151), .c(new_n152), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n132), .b(new_n153), .out0(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n150), .c(new_n146), .d(new_n149), .o1(new_n155));
  oai022aa1d18x5               g060(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n156));
  aoai13aa1n06x5               g061(.a(new_n136), .b(new_n135), .c(new_n156), .d(new_n131), .o1(new_n157));
  oaoi03aa1n12x5               g062(.a(\a[12] ), .b(\b[11] ), .c(new_n157), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(new_n155), .b(new_n159), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g066(.a(\a[13] ), .o1(new_n162));
  inv000aa1d42x5               g067(.a(\b[12] ), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(new_n162), .b(new_n163), .c(new_n160), .o1(new_n164));
  xnrb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  nand02aa1n03x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  nor002aa1n16x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand02aa1n08x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nona23aa1n12x5               g074(.a(new_n169), .b(new_n167), .c(new_n166), .d(new_n168), .out0(new_n170));
  aoai13aa1n04x5               g075(.a(new_n169), .b(new_n168), .c(new_n162), .d(new_n163), .o1(new_n171));
  aoai13aa1n04x5               g076(.a(new_n171), .b(new_n170), .c(new_n155), .d(new_n159), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n06x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nand42aa1d28x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  norp02aa1n04x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanp02aa1n09x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  aoai13aa1n03x5               g083(.a(new_n178), .b(new_n174), .c(new_n172), .d(new_n175), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(new_n174), .b(new_n178), .c(new_n172), .d(new_n175), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(\s[16] ));
  nano23aa1n06x5               g086(.a(new_n174), .b(new_n176), .c(new_n177), .d(new_n175), .out0(new_n182));
  nano23aa1n06x5               g087(.a(new_n170), .b(new_n153), .c(new_n182), .d(new_n132), .out0(new_n183));
  aoai13aa1n12x5               g088(.a(new_n183), .b(new_n150), .c(new_n146), .d(new_n149), .o1(new_n184));
  inv000aa1n02x5               g089(.a(new_n174), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n175), .o1(new_n186));
  inv000aa1n02x5               g091(.a(new_n176), .o1(new_n187));
  aoai13aa1n04x5               g092(.a(new_n187), .b(new_n186), .c(new_n171), .d(new_n185), .o1(new_n188));
  norb02aa1n02x7               g093(.a(new_n175), .b(new_n174), .out0(new_n189));
  nano32aa1n03x7               g094(.a(new_n170), .b(new_n177), .c(new_n189), .d(new_n187), .out0(new_n190));
  aoi022aa1d18x5               g095(.a(new_n158), .b(new_n190), .c(new_n177), .d(new_n188), .o1(new_n191));
  xorc02aa1n12x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n192), .b(new_n184), .c(new_n191), .out0(\s[17] ));
  inv000aa1d42x5               g098(.a(\a[18] ), .o1(new_n194));
  nanp02aa1n09x5               g099(.a(new_n184), .b(new_n191), .o1(new_n195));
  nor042aa1n06x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  tech160nm_fiaoi012aa1n05x5   g101(.a(new_n196), .b(new_n195), .c(new_n192), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(new_n194), .out0(\s[18] ));
  tech160nm_fixorc02aa1n04x5   g103(.a(\a[18] ), .b(\b[17] ), .out0(new_n199));
  and002aa1n12x5               g104(.a(new_n199), .b(new_n192), .o(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  oab012aa1n06x5               g107(.a(new_n196), .b(\a[18] ), .c(\b[17] ), .out0(new_n203));
  norb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  aoai13aa1n04x5               g110(.a(new_n205), .b(new_n201), .c(new_n184), .d(new_n191), .o1(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nand42aa1n02x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  xnrc02aa1n02x5               g115(.a(\b[19] ), .b(\a[20] ), .out0(new_n211));
  aoai13aa1n03x5               g116(.a(new_n211), .b(new_n209), .c(new_n206), .d(new_n210), .o1(new_n212));
  nanb02aa1n02x5               g117(.a(new_n209), .b(new_n210), .out0(new_n213));
  nanb02aa1n03x5               g118(.a(new_n213), .b(new_n206), .out0(new_n214));
  nona22aa1n02x5               g119(.a(new_n214), .b(new_n211), .c(new_n209), .out0(new_n215));
  nand42aa1n02x5               g120(.a(new_n215), .b(new_n212), .o1(\s[20] ));
  nona22aa1n06x5               g121(.a(new_n200), .b(new_n213), .c(new_n211), .out0(new_n217));
  nanp02aa1n02x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  oai022aa1n12x5               g123(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n219));
  nano22aa1n03x7               g124(.a(new_n203), .b(new_n202), .c(new_n210), .out0(new_n220));
  oai012aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n219), .o1(new_n221));
  aoai13aa1n04x5               g126(.a(new_n221), .b(new_n217), .c(new_n184), .d(new_n191), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  inv000aa1d42x5               g128(.a(\a[21] ), .o1(new_n224));
  nanb02aa1d36x5               g129(.a(\b[20] ), .b(new_n224), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[21] ), .b(\b[20] ), .out0(new_n227));
  xnrc02aa1n06x5               g132(.a(\b[21] ), .b(\a[22] ), .out0(new_n228));
  aoai13aa1n03x5               g133(.a(new_n228), .b(new_n226), .c(new_n222), .d(new_n227), .o1(new_n229));
  nanp02aa1n02x5               g134(.a(new_n222), .b(new_n227), .o1(new_n230));
  nona22aa1n02x5               g135(.a(new_n230), .b(new_n228), .c(new_n226), .out0(new_n231));
  nanp02aa1n03x5               g136(.a(new_n231), .b(new_n229), .o1(\s[22] ));
  nanp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  nano22aa1n02x4               g138(.a(new_n228), .b(new_n225), .c(new_n233), .out0(new_n234));
  nona23aa1n02x4               g139(.a(new_n200), .b(new_n234), .c(new_n211), .d(new_n213), .out0(new_n235));
  nano32aa1n03x7               g140(.a(new_n228), .b(new_n233), .c(new_n225), .d(new_n218), .out0(new_n236));
  tech160nm_fioaoi03aa1n02p5x5 g141(.a(\a[22] ), .b(\b[21] ), .c(new_n225), .o1(new_n237));
  oaoi13aa1n02x5               g142(.a(new_n237), .b(new_n236), .c(new_n220), .d(new_n219), .o1(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n235), .c(new_n184), .d(new_n191), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  tech160nm_fixorc02aa1n05x5   g146(.a(\a[23] ), .b(\b[22] ), .out0(new_n242));
  nor002aa1n03x5               g147(.a(\b[23] ), .b(\a[24] ), .o1(new_n243));
  nand42aa1n03x5               g148(.a(\b[23] ), .b(\a[24] ), .o1(new_n244));
  nanb02aa1n06x5               g149(.a(new_n243), .b(new_n244), .out0(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n241), .c(new_n239), .d(new_n242), .o1(new_n246));
  nand42aa1n02x5               g151(.a(new_n239), .b(new_n242), .o1(new_n247));
  nona22aa1n02x5               g152(.a(new_n247), .b(new_n245), .c(new_n241), .out0(new_n248));
  nanp02aa1n03x5               g153(.a(new_n248), .b(new_n246), .o1(\s[24] ));
  norb02aa1n06x4               g154(.a(new_n242), .b(new_n245), .out0(new_n250));
  nanb03aa1n02x5               g155(.a(new_n217), .b(new_n250), .c(new_n234), .out0(new_n251));
  oai012aa1n06x5               g156(.a(new_n236), .b(new_n220), .c(new_n219), .o1(new_n252));
  inv020aa1n04x5               g157(.a(new_n237), .o1(new_n253));
  inv000aa1n02x5               g158(.a(new_n250), .o1(new_n254));
  tech160nm_fioai012aa1n03p5x5 g159(.a(new_n244), .b(new_n243), .c(new_n241), .o1(new_n255));
  aoai13aa1n12x5               g160(.a(new_n255), .b(new_n254), .c(new_n252), .d(new_n253), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  aoai13aa1n04x5               g162(.a(new_n257), .b(new_n251), .c(new_n184), .d(new_n191), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  tech160nm_fixorc02aa1n05x5   g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  nor042aa1n02x5               g166(.a(\b[25] ), .b(\a[26] ), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(\b[25] ), .b(\a[26] ), .o1(new_n263));
  nanb02aa1n06x5               g168(.a(new_n262), .b(new_n263), .out0(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n260), .c(new_n258), .d(new_n261), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(new_n258), .b(new_n261), .o1(new_n266));
  nona22aa1n03x5               g171(.a(new_n266), .b(new_n264), .c(new_n260), .out0(new_n267));
  nanp02aa1n03x5               g172(.a(new_n267), .b(new_n265), .o1(\s[26] ));
  nanp02aa1n02x5               g173(.a(new_n188), .b(new_n177), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n136), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n151), .b(new_n270), .c(new_n133), .d(new_n141), .o1(new_n271));
  nand03aa1n02x5               g176(.a(new_n190), .b(new_n271), .c(new_n152), .o1(new_n272));
  nand42aa1n03x5               g177(.a(new_n272), .b(new_n269), .o1(new_n273));
  norb02aa1n03x5               g178(.a(new_n261), .b(new_n264), .out0(new_n274));
  nano32aa1n03x7               g179(.a(new_n217), .b(new_n274), .c(new_n234), .d(new_n250), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n273), .c(new_n125), .d(new_n183), .o1(new_n276));
  oai012aa1n02x5               g181(.a(new_n263), .b(new_n262), .c(new_n260), .o1(new_n277));
  aobi12aa1n09x5               g182(.a(new_n277), .b(new_n256), .c(new_n274), .out0(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n276), .c(new_n278), .out0(\s[27] ));
  nand02aa1n03x5               g185(.a(new_n276), .b(new_n278), .o1(new_n281));
  nor042aa1n03x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[28] ), .b(\b[27] ), .out0(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n282), .c(new_n281), .d(new_n279), .o1(new_n285));
  oai022aa1n02x5               g190(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n286));
  nanp03aa1n02x5               g191(.a(new_n286), .b(new_n202), .c(new_n210), .o1(new_n287));
  nanb02aa1n02x5               g192(.a(new_n219), .b(new_n287), .out0(new_n288));
  aoai13aa1n02x5               g193(.a(new_n250), .b(new_n237), .c(new_n288), .d(new_n236), .o1(new_n289));
  inv000aa1n02x5               g194(.a(new_n274), .o1(new_n290));
  aoai13aa1n09x5               g195(.a(new_n277), .b(new_n290), .c(new_n289), .d(new_n255), .o1(new_n291));
  aoai13aa1n03x5               g196(.a(new_n279), .b(new_n291), .c(new_n195), .d(new_n275), .o1(new_n292));
  nona22aa1n02x5               g197(.a(new_n292), .b(new_n284), .c(new_n282), .out0(new_n293));
  nanp02aa1n03x5               g198(.a(new_n285), .b(new_n293), .o1(\s[28] ));
  xorc02aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .out0(new_n295));
  aobi12aa1n06x5               g200(.a(new_n275), .b(new_n184), .c(new_n191), .out0(new_n296));
  and002aa1n02x5               g201(.a(new_n283), .b(new_n279), .o(new_n297));
  inv000aa1d42x5               g202(.a(\a[28] ), .o1(new_n298));
  inv000aa1d42x5               g203(.a(\b[27] ), .o1(new_n299));
  oaoi03aa1n09x5               g204(.a(new_n298), .b(new_n299), .c(new_n282), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n300), .o1(new_n301));
  oaoi13aa1n03x5               g206(.a(new_n301), .b(new_n297), .c(new_n296), .d(new_n291), .o1(new_n302));
  aoai13aa1n03x5               g207(.a(new_n297), .b(new_n291), .c(new_n195), .d(new_n275), .o1(new_n303));
  nanp03aa1n03x5               g208(.a(new_n303), .b(new_n295), .c(new_n300), .o1(new_n304));
  oai012aa1n03x5               g209(.a(new_n304), .b(new_n302), .c(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g210(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g211(.a(new_n284), .b(new_n279), .c(new_n295), .out0(new_n307));
  tech160nm_fioaoi03aa1n03p5x5 g212(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .o1(new_n308));
  oaoi13aa1n03x5               g213(.a(new_n308), .b(new_n307), .c(new_n296), .d(new_n291), .o1(new_n309));
  xorc02aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .out0(new_n310));
  inv000aa1d42x5               g215(.a(new_n307), .o1(new_n311));
  norb02aa1n02x5               g216(.a(new_n310), .b(new_n308), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n311), .c(new_n276), .d(new_n278), .o1(new_n313));
  oai012aa1n03x5               g218(.a(new_n313), .b(new_n309), .c(new_n310), .o1(\s[30] ));
  and003aa1n02x5               g219(.a(new_n297), .b(new_n295), .c(new_n310), .o(new_n315));
  aoi012aa1n02x7               g220(.a(new_n312), .b(\a[30] ), .c(\b[29] ), .o1(new_n316));
  xnrc02aa1n02x5               g221(.a(\b[30] ), .b(\a[31] ), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n316), .c(new_n281), .d(new_n315), .o1(new_n318));
  aoai13aa1n03x5               g223(.a(new_n315), .b(new_n291), .c(new_n195), .d(new_n275), .o1(new_n319));
  nona22aa1n02x5               g224(.a(new_n319), .b(new_n316), .c(new_n317), .out0(new_n320));
  nanp02aa1n03x5               g225(.a(new_n318), .b(new_n320), .o1(\s[31] ));
  xnbna2aa1n03x5               g226(.a(new_n102), .b(new_n105), .c(new_n108), .out0(\s[3] ));
  aoai13aa1n02x5               g227(.a(new_n106), .b(new_n99), .c(new_n101), .d(new_n100), .o1(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n103), .b(new_n323), .c(new_n108), .out0(\s[4] ));
  xorb03aa1n02x5               g229(.a(new_n146), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g230(.a(new_n117), .b(new_n146), .c(new_n118), .o1(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g232(.a(new_n121), .b(new_n119), .c(new_n107), .d(new_n109), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoai13aa1n02x5               g234(.a(new_n147), .b(new_n112), .c(new_n328), .d(new_n113), .o1(new_n330));
  aoi112aa1n02x5               g235(.a(new_n147), .b(new_n112), .c(new_n328), .d(new_n113), .o1(new_n331));
  nanb02aa1n02x5               g236(.a(new_n331), .b(new_n330), .out0(\s[8] ));
  xorb03aa1n02x5               g237(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


