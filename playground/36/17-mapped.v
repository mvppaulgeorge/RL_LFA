// Benchmark "adder" written by ABC on Thu Jul 18 06:32:21 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n323, new_n326,
    new_n328, new_n330, new_n331;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d06x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand02aa1d06x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n12x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  inv040aa1d32x5               g007(.a(\a[4] ), .o1(new_n103));
  inv040aa1d32x5               g008(.a(\b[3] ), .o1(new_n104));
  nanp02aa1n09x5               g009(.a(new_n104), .b(new_n103), .o1(new_n105));
  nand42aa1n06x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand42aa1n06x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  inv040aa1d32x5               g012(.a(\a[3] ), .o1(new_n108));
  inv040aa1d28x5               g013(.a(\b[2] ), .o1(new_n109));
  nand42aa1d28x5               g014(.a(new_n109), .b(new_n108), .o1(new_n110));
  nand42aa1n06x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nand42aa1n06x5               g016(.a(new_n110), .b(new_n111), .o1(new_n112));
  oaoi03aa1n09x5               g017(.a(\a[4] ), .b(\b[3] ), .c(new_n110), .o1(new_n113));
  inv000aa1n02x5               g018(.a(new_n113), .o1(new_n114));
  oai013aa1d12x5               g019(.a(new_n114), .b(new_n102), .c(new_n107), .d(new_n112), .o1(new_n115));
  nor022aa1n16x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand42aa1n10x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nanb02aa1d36x5               g022(.a(new_n116), .b(new_n117), .out0(new_n118));
  xnrc02aa1n12x5               g023(.a(\b[6] ), .b(\a[7] ), .out0(new_n119));
  nand02aa1d28x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  nor002aa1d32x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  nor002aa1d24x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  tech160nm_finand02aa1n03p5x5 g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  nona23aa1n09x5               g028(.a(new_n120), .b(new_n123), .c(new_n122), .d(new_n121), .out0(new_n124));
  nor043aa1n04x5               g029(.a(new_n124), .b(new_n119), .c(new_n118), .o1(new_n125));
  aoi012aa1d18x5               g030(.a(new_n121), .b(new_n122), .c(new_n120), .o1(new_n126));
  nor022aa1n12x5               g031(.a(\b[6] ), .b(\a[7] ), .o1(new_n127));
  tech160nm_fioai012aa1n04x5   g032(.a(new_n117), .b(new_n127), .c(new_n116), .o1(new_n128));
  oai013aa1d12x5               g033(.a(new_n128), .b(new_n119), .c(new_n126), .d(new_n118), .o1(new_n129));
  nand42aa1n06x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n97), .out0(new_n131));
  aoai13aa1n02x5               g036(.a(new_n131), .b(new_n129), .c(new_n115), .d(new_n125), .o1(new_n132));
  nor042aa1n09x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nand42aa1n08x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  nona23aa1n02x4               g040(.a(new_n132), .b(new_n134), .c(new_n133), .d(new_n97), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n135), .c(new_n98), .d(new_n132), .o1(\s[10] ));
  nano23aa1n03x7               g042(.a(new_n97), .b(new_n133), .c(new_n134), .d(new_n130), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n129), .c(new_n115), .d(new_n125), .o1(new_n139));
  tech160nm_fioai012aa1n03p5x5 g044(.a(new_n134), .b(new_n133), .c(new_n97), .o1(new_n140));
  tech160nm_fixorc02aa1n03p5x5 g045(.a(\a[11] ), .b(\b[10] ), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n139), .c(new_n140), .out0(\s[11] ));
  orn002aa1n02x5               g047(.a(\a[11] ), .b(\b[10] ), .o(new_n143));
  and002aa1n02x5               g048(.a(\b[10] ), .b(\a[11] ), .o(new_n144));
  aoai13aa1n02x5               g049(.a(new_n143), .b(new_n144), .c(new_n139), .d(new_n140), .o1(new_n145));
  xorb03aa1n02x5               g050(.a(new_n145), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  xorc02aa1n02x5               g051(.a(\a[12] ), .b(\b[11] ), .out0(new_n147));
  and003aa1n06x5               g052(.a(new_n138), .b(new_n147), .c(new_n141), .o(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n129), .c(new_n115), .d(new_n125), .o1(new_n149));
  inv000aa1d42x5               g054(.a(\b[11] ), .o1(new_n150));
  nanb02aa1n02x5               g055(.a(\a[12] ), .b(new_n150), .out0(new_n151));
  aoai13aa1n09x5               g056(.a(new_n151), .b(new_n144), .c(new_n140), .d(new_n143), .o1(new_n152));
  oaib12aa1n02x5               g057(.a(new_n152), .b(new_n150), .c(\a[12] ), .out0(new_n153));
  nor042aa1n06x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand02aa1d06x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n149), .c(new_n153), .out0(\s[13] ));
  norp03aa1n02x5               g062(.a(new_n102), .b(new_n107), .c(new_n112), .o1(new_n158));
  oai012aa1n02x5               g063(.a(new_n125), .b(new_n158), .c(new_n113), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n129), .o1(new_n160));
  inv000aa1n02x5               g065(.a(new_n148), .o1(new_n161));
  aoai13aa1n06x5               g066(.a(new_n153), .b(new_n161), .c(new_n159), .d(new_n160), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n154), .b(new_n162), .c(new_n155), .o1(new_n163));
  nor042aa1n04x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand02aa1d16x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  oaih22aa1d12x5               g071(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n167));
  aoi122aa1n02x5               g072(.a(new_n167), .b(\b[13] ), .c(\a[14] ), .d(new_n162), .e(new_n156), .o1(new_n168));
  oabi12aa1n02x5               g073(.a(new_n168), .b(new_n163), .c(new_n166), .out0(\s[14] ));
  nano23aa1n06x5               g074(.a(new_n154), .b(new_n164), .c(new_n165), .d(new_n155), .out0(new_n170));
  inv040aa1n02x5               g075(.a(new_n170), .o1(new_n171));
  oaih12aa1n12x5               g076(.a(new_n165), .b(new_n164), .c(new_n154), .o1(new_n172));
  aoai13aa1n03x5               g077(.a(new_n172), .b(new_n171), .c(new_n149), .d(new_n153), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n12x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  inv000aa1d42x5               g081(.a(new_n172), .o1(new_n177));
  nand42aa1n04x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  norb02aa1n03x4               g083(.a(new_n178), .b(new_n175), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n177), .c(new_n162), .d(new_n170), .o1(new_n180));
  nor042aa1n06x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nand02aa1n10x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  norb02aa1n15x5               g087(.a(new_n182), .b(new_n181), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  tech160nm_fiaoi012aa1n02p5x5 g089(.a(new_n184), .b(new_n180), .c(new_n176), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n175), .b(new_n183), .c(new_n173), .d(new_n178), .o1(new_n186));
  norp02aa1n02x5               g091(.a(new_n185), .b(new_n186), .o1(\s[16] ));
  nano23aa1n02x5               g092(.a(new_n175), .b(new_n181), .c(new_n182), .d(new_n178), .out0(new_n188));
  nand02aa1n02x5               g093(.a(new_n188), .b(new_n170), .o1(new_n189));
  nano32aa1n03x7               g094(.a(new_n189), .b(new_n138), .c(new_n141), .d(new_n147), .out0(new_n190));
  aoai13aa1n12x5               g095(.a(new_n190), .b(new_n129), .c(new_n115), .d(new_n125), .o1(new_n191));
  aoi022aa1n02x5               g096(.a(\b[15] ), .b(\a[16] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n192));
  nano32aa1n03x7               g097(.a(new_n171), .b(new_n192), .c(new_n179), .d(new_n183), .out0(new_n193));
  aoai13aa1n02x7               g098(.a(new_n178), .b(new_n175), .c(new_n167), .d(new_n165), .o1(new_n194));
  oai012aa1n03x5               g099(.a(new_n194), .b(\b[15] ), .c(\a[16] ), .o1(new_n195));
  aoi022aa1n12x5               g100(.a(new_n193), .b(new_n152), .c(new_n182), .d(new_n195), .o1(new_n196));
  xorc02aa1n02x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n191), .c(new_n196), .out0(\s[17] ));
  nor002aa1n16x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  nanp02aa1n12x5               g105(.a(new_n191), .b(new_n196), .o1(new_n201));
  nand22aa1n03x5               g106(.a(new_n201), .b(new_n197), .o1(new_n202));
  xorc02aa1n02x5               g107(.a(\a[18] ), .b(\b[17] ), .out0(new_n203));
  nand42aa1n20x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  oai022aa1d18x5               g109(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n205));
  nanb03aa1n02x5               g110(.a(new_n205), .b(new_n202), .c(new_n204), .out0(new_n206));
  aoai13aa1n02x5               g111(.a(new_n206), .b(new_n203), .c(new_n200), .d(new_n202), .o1(\s[18] ));
  nand42aa1d28x5               g112(.a(\b[16] ), .b(\a[17] ), .o1(new_n208));
  nor002aa1n03x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  nano23aa1d15x5               g114(.a(new_n199), .b(new_n209), .c(new_n204), .d(new_n208), .out0(new_n210));
  oaoi03aa1n02x5               g115(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n211));
  xorc02aa1n12x5               g116(.a(\a[19] ), .b(\b[18] ), .out0(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n211), .c(new_n201), .d(new_n210), .o1(new_n213));
  aoi112aa1n02x5               g118(.a(new_n212), .b(new_n211), .c(new_n201), .d(new_n210), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1d32x5               g121(.a(\a[19] ), .o1(new_n217));
  inv000aa1d42x5               g122(.a(\b[18] ), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(new_n218), .b(new_n217), .o1(new_n219));
  tech160nm_fixorc02aa1n03p5x5 g124(.a(\a[20] ), .b(\b[19] ), .out0(new_n220));
  nand02aa1n08x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  oai022aa1n02x5               g127(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n223));
  nona22aa1n03x5               g128(.a(new_n213), .b(new_n222), .c(new_n223), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n220), .c(new_n219), .d(new_n213), .o1(\s[20] ));
  nanp03aa1d12x5               g130(.a(new_n210), .b(new_n212), .c(new_n220), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  oai112aa1n02x5               g132(.a(new_n205), .b(new_n204), .c(new_n218), .d(new_n217), .o1(new_n228));
  aoib12aa1n02x5               g133(.a(new_n222), .b(new_n228), .c(new_n223), .out0(new_n229));
  orn002aa1n24x5               g134(.a(\a[21] ), .b(\b[20] ), .o(new_n230));
  nand42aa1n06x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  nanp02aa1n09x5               g136(.a(new_n230), .b(new_n231), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n229), .c(new_n201), .d(new_n227), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(new_n233), .b(new_n229), .c(new_n201), .d(new_n227), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n234), .b(new_n235), .out0(\s[21] ));
  xnrc02aa1n12x5               g141(.a(\b[21] ), .b(\a[22] ), .out0(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  and002aa1n02x5               g143(.a(\b[21] ), .b(\a[22] ), .o(new_n239));
  oai022aa1n02x5               g144(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n240));
  nona22aa1n02x5               g145(.a(new_n234), .b(new_n239), .c(new_n240), .out0(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n238), .c(new_n230), .d(new_n234), .o1(\s[22] ));
  norp02aa1n02x5               g147(.a(new_n237), .b(new_n232), .o1(new_n243));
  norb02aa1n09x5               g148(.a(new_n243), .b(new_n226), .out0(new_n244));
  nanb02aa1n03x5               g149(.a(new_n223), .b(new_n228), .out0(new_n245));
  nano32aa1n03x7               g150(.a(new_n237), .b(new_n230), .c(new_n231), .d(new_n221), .out0(new_n246));
  nanp02aa1n02x5               g151(.a(new_n245), .b(new_n246), .o1(new_n247));
  oaib12aa1n02x5               g152(.a(new_n247), .b(new_n239), .c(new_n240), .out0(new_n248));
  xorc02aa1n12x5               g153(.a(\a[23] ), .b(\b[22] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n248), .c(new_n201), .d(new_n244), .o1(new_n250));
  oaoi03aa1n02x5               g155(.a(\a[22] ), .b(\b[21] ), .c(new_n230), .o1(new_n251));
  nona22aa1n02x4               g156(.a(new_n247), .b(new_n251), .c(new_n249), .out0(new_n252));
  aoi012aa1n02x5               g157(.a(new_n252), .b(new_n201), .c(new_n244), .o1(new_n253));
  norb02aa1n02x5               g158(.a(new_n250), .b(new_n253), .out0(\s[23] ));
  norp02aa1n02x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  tech160nm_fixorc02aa1n04x5   g161(.a(\a[24] ), .b(\b[23] ), .out0(new_n257));
  and002aa1n02x5               g162(.a(\b[23] ), .b(\a[24] ), .o(new_n258));
  oai022aa1n02x5               g163(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n259));
  nona22aa1n02x5               g164(.a(new_n250), .b(new_n258), .c(new_n259), .out0(new_n260));
  aoai13aa1n03x5               g165(.a(new_n260), .b(new_n257), .c(new_n256), .d(new_n250), .o1(\s[24] ));
  nano32aa1n06x5               g166(.a(new_n226), .b(new_n257), .c(new_n243), .d(new_n249), .out0(new_n262));
  aob012aa1n02x5               g167(.a(new_n259), .b(\b[23] ), .c(\a[24] ), .out0(new_n263));
  and002aa1n06x5               g168(.a(new_n257), .b(new_n249), .o(new_n264));
  aoai13aa1n04x5               g169(.a(new_n264), .b(new_n251), .c(new_n245), .d(new_n246), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(new_n265), .b(new_n263), .o1(new_n266));
  xnrc02aa1n12x5               g171(.a(\b[24] ), .b(\a[25] ), .out0(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n266), .c(new_n201), .d(new_n262), .o1(new_n269));
  aoi112aa1n02x5               g174(.a(new_n268), .b(new_n266), .c(new_n201), .d(new_n262), .o1(new_n270));
  norb02aa1n02x5               g175(.a(new_n269), .b(new_n270), .out0(\s[25] ));
  norp02aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .out0(new_n274));
  and002aa1n02x5               g179(.a(\b[25] ), .b(\a[26] ), .o(new_n275));
  oai022aa1n02x5               g180(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n276));
  nona22aa1n02x5               g181(.a(new_n269), .b(new_n275), .c(new_n276), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n274), .c(new_n273), .d(new_n269), .o1(\s[26] ));
  norb02aa1n09x5               g183(.a(new_n274), .b(new_n267), .out0(new_n279));
  nand23aa1n06x5               g184(.a(new_n244), .b(new_n264), .c(new_n279), .o1(new_n280));
  inv040aa1n06x5               g185(.a(new_n280), .o1(new_n281));
  inv000aa1n02x5               g186(.a(new_n279), .o1(new_n282));
  aob012aa1n02x5               g187(.a(new_n276), .b(\b[25] ), .c(\a[26] ), .out0(new_n283));
  aoai13aa1n12x5               g188(.a(new_n283), .b(new_n282), .c(new_n265), .d(new_n263), .o1(new_n284));
  xorc02aa1n02x5               g189(.a(\a[27] ), .b(\b[26] ), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n284), .c(new_n201), .d(new_n281), .o1(new_n286));
  aoi112aa1n02x5               g191(.a(new_n284), .b(new_n285), .c(new_n201), .d(new_n281), .o1(new_n287));
  norb02aa1n02x7               g192(.a(new_n286), .b(new_n287), .out0(\s[27] ));
  norp02aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  inv020aa1n02x5               g194(.a(new_n289), .o1(new_n290));
  xorc02aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .out0(new_n291));
  and002aa1n02x5               g196(.a(\b[27] ), .b(\a[28] ), .o(new_n292));
  oai022aa1n02x5               g197(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n293));
  nona22aa1n02x5               g198(.a(new_n286), .b(new_n292), .c(new_n293), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n291), .c(new_n290), .d(new_n286), .o1(\s[28] ));
  xorc02aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .out0(new_n296));
  tech160nm_fiaoi012aa1n03p5x5 g201(.a(new_n280), .b(new_n191), .c(new_n196), .o1(new_n297));
  and002aa1n02x5               g202(.a(new_n291), .b(new_n285), .o(new_n298));
  oaoi03aa1n02x5               g203(.a(\a[28] ), .b(\b[27] ), .c(new_n290), .o1(new_n299));
  oaoi13aa1n03x5               g204(.a(new_n299), .b(new_n298), .c(new_n297), .d(new_n284), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n298), .b(new_n284), .c(new_n201), .d(new_n281), .o1(new_n301));
  norb02aa1n02x5               g206(.a(new_n296), .b(new_n299), .out0(new_n302));
  nand42aa1n02x5               g207(.a(new_n301), .b(new_n302), .o1(new_n303));
  oai012aa1n03x5               g208(.a(new_n303), .b(new_n300), .c(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g209(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g210(.a(new_n285), .b(new_n296), .c(new_n291), .o(new_n306));
  aoi012aa1n02x5               g211(.a(new_n302), .b(\a[29] ), .c(\b[28] ), .o1(new_n307));
  oaoi13aa1n03x5               g212(.a(new_n307), .b(new_n306), .c(new_n297), .d(new_n284), .o1(new_n308));
  xorc02aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n306), .b(new_n284), .c(new_n201), .d(new_n281), .o1(new_n310));
  nanb03aa1n03x5               g215(.a(new_n307), .b(new_n310), .c(new_n309), .out0(new_n311));
  oai012aa1n03x5               g216(.a(new_n311), .b(new_n308), .c(new_n309), .o1(\s[30] ));
  and003aa1n02x5               g217(.a(new_n298), .b(new_n296), .c(new_n309), .o(new_n313));
  aoai13aa1n06x5               g218(.a(new_n313), .b(new_n284), .c(new_n201), .d(new_n281), .o1(new_n314));
  inv000aa1d42x5               g219(.a(\a[30] ), .o1(new_n315));
  and002aa1n02x5               g220(.a(new_n307), .b(new_n309), .o(new_n316));
  aoib12aa1n02x5               g221(.a(new_n316), .b(new_n315), .c(\b[29] ), .out0(new_n317));
  xorc02aa1n02x5               g222(.a(\a[31] ), .b(\b[30] ), .out0(new_n318));
  oaib12aa1n02x5               g223(.a(new_n318), .b(\b[29] ), .c(new_n315), .out0(new_n319));
  nona22aa1n02x5               g224(.a(new_n314), .b(new_n316), .c(new_n319), .out0(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n318), .c(new_n314), .d(new_n317), .o1(\s[31] ));
  xnbna2aa1n03x5               g226(.a(new_n102), .b(new_n111), .c(new_n110), .out0(\s[3] ));
  orn002aa1n02x5               g227(.a(new_n102), .b(new_n112), .o(new_n323));
  xobna2aa1n03x5               g228(.a(new_n107), .b(new_n323), .c(new_n110), .out0(\s[4] ));
  xorb03aa1n02x5               g229(.a(new_n115), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g230(.a(new_n122), .b(new_n115), .c(new_n123), .o1(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g232(.a(new_n126), .b(new_n124), .c(new_n115), .out0(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  inv000aa1d42x5               g234(.a(new_n127), .o1(new_n330));
  nanb02aa1n02x5               g235(.a(new_n119), .b(new_n328), .out0(new_n331));
  xobna2aa1n03x5               g236(.a(new_n118), .b(new_n331), .c(new_n330), .out0(\s[8] ));
  xnbna2aa1n03x5               g237(.a(new_n131), .b(new_n159), .c(new_n160), .out0(\s[9] ));
endmodule


