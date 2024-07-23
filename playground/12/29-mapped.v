// Benchmark "adder" written by ABC on Wed Jul 17 18:21:13 2024

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
    new_n118, new_n119, new_n120, new_n122, new_n123, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n309,
    new_n311, new_n312, new_n313, new_n316, new_n317, new_n319, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nand02aa1d08x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nor002aa1d32x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nand22aa1n04x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nona23aa1d18x5               g005(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n101));
  inv040aa1n06x5               g006(.a(new_n101), .o1(new_n102));
  oa0022aa1n06x5               g007(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n103));
  xnrc02aa1n12x5               g008(.a(\b[2] ), .b(\a[3] ), .out0(new_n104));
  nand42aa1n06x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nor042aa1n09x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nand02aa1d28x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  oai012aa1n12x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o1(new_n108));
  oai012aa1d24x5               g013(.a(new_n103), .b(new_n104), .c(new_n108), .o1(new_n109));
  tech160nm_finand02aa1n03p5x5 g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  orn002aa1n24x5               g015(.a(\a[6] ), .b(\b[5] ), .o(new_n111));
  nand22aa1n12x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  xnrc02aa1n12x5               g017(.a(\b[4] ), .b(\a[5] ), .out0(new_n113));
  nano32aa1n09x5               g018(.a(new_n113), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n114));
  nor042aa1n09x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  aob012aa1n12x5               g020(.a(new_n111), .b(new_n115), .c(new_n112), .out0(new_n116));
  tech160nm_fioai012aa1n03p5x5 g021(.a(new_n98), .b(new_n99), .c(new_n97), .o1(new_n117));
  oaib12aa1n18x5               g022(.a(new_n117), .b(new_n101), .c(new_n116), .out0(new_n118));
  aoi013aa1n09x5               g023(.a(new_n118), .b(new_n109), .c(new_n114), .d(new_n102), .o1(new_n119));
  oaoi03aa1n09x5               g024(.a(\a[9] ), .b(\b[8] ), .c(new_n119), .o1(new_n120));
  xorb03aa1n02x5               g025(.a(new_n120), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n06x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  nand23aa1n04x5               g027(.a(new_n109), .b(new_n102), .c(new_n114), .o1(new_n123));
  aobi12aa1n06x5               g028(.a(new_n117), .b(new_n102), .c(new_n116), .out0(new_n124));
  nanp02aa1n03x5               g029(.a(new_n123), .b(new_n124), .o1(new_n125));
  xorc02aa1n06x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  xorc02aa1n12x5               g031(.a(\a[10] ), .b(\b[9] ), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n122), .c(new_n125), .d(new_n126), .o1(new_n128));
  inv000aa1d42x5               g033(.a(\a[10] ), .o1(new_n129));
  inv000aa1d42x5               g034(.a(\b[9] ), .o1(new_n130));
  tech160nm_fioaoi03aa1n04x5   g035(.a(new_n129), .b(new_n130), .c(new_n122), .o1(new_n131));
  tech160nm_fixorc02aa1n03p5x5 g036(.a(\a[11] ), .b(\b[10] ), .out0(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n128), .c(new_n131), .out0(\s[11] ));
  and002aa1n09x5               g038(.a(\b[10] ), .b(\a[11] ), .o(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  oao003aa1n06x5               g040(.a(new_n129), .b(new_n130), .c(new_n122), .carry(new_n136));
  tech160nm_fixnrc02aa1n05x5   g041(.a(\b[10] ), .b(\a[11] ), .out0(new_n137));
  aoi112aa1n06x5               g042(.a(new_n137), .b(new_n136), .c(new_n120), .d(new_n127), .o1(new_n138));
  xorc02aa1n12x5               g043(.a(\a[12] ), .b(\b[11] ), .out0(new_n139));
  nona22aa1n09x5               g044(.a(new_n135), .b(new_n138), .c(new_n139), .out0(new_n140));
  oai012aa1n03x5               g045(.a(new_n139), .b(new_n138), .c(new_n134), .o1(new_n141));
  nanp02aa1n03x5               g046(.a(new_n140), .b(new_n141), .o1(\s[12] ));
  xnrc02aa1n02x5               g047(.a(\b[8] ), .b(\a[9] ), .out0(new_n143));
  nona23aa1n03x5               g048(.a(new_n139), .b(new_n127), .c(new_n143), .d(new_n137), .out0(new_n144));
  and002aa1n06x5               g049(.a(\b[11] ), .b(\a[12] ), .o(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  oa0022aa1n06x5               g051(.a(\a[12] ), .b(\b[11] ), .c(\a[11] ), .d(\b[10] ), .o(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  aoai13aa1n04x5               g053(.a(new_n146), .b(new_n148), .c(new_n136), .d(new_n135), .o1(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n144), .c(new_n123), .d(new_n124), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1d18x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand42aa1d28x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .o1(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n06x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand42aa1n20x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nano23aa1d15x5               g062(.a(new_n152), .b(new_n156), .c(new_n157), .d(new_n153), .out0(new_n158));
  tech160nm_fiao0012aa1n02p5x5 g063(.a(new_n156), .b(new_n152), .c(new_n157), .o(new_n159));
  xorc02aa1n12x5               g064(.a(\a[15] ), .b(\b[14] ), .out0(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n159), .c(new_n150), .d(new_n158), .o1(new_n161));
  aoi112aa1n02x5               g066(.a(new_n160), .b(new_n159), .c(new_n150), .d(new_n158), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(\s[15] ));
  orn002aa1n02x5               g068(.a(\a[15] ), .b(\b[14] ), .o(new_n164));
  xorc02aa1n12x5               g069(.a(\a[16] ), .b(\b[15] ), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n161), .c(new_n164), .out0(\s[16] ));
  nand02aa1n03x5               g071(.a(new_n139), .b(new_n126), .o1(new_n167));
  nano22aa1n03x7               g072(.a(new_n167), .b(new_n127), .c(new_n132), .out0(new_n168));
  nand23aa1n09x5               g073(.a(new_n158), .b(new_n160), .c(new_n165), .o1(new_n169));
  nanb02aa1d24x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  oaoi13aa1n12x5               g075(.a(new_n145), .b(new_n147), .c(new_n131), .d(new_n134), .o1(new_n171));
  inv040aa1n02x5               g076(.a(new_n169), .o1(new_n172));
  inv000aa1d42x5               g077(.a(\a[16] ), .o1(new_n173));
  inv000aa1d42x5               g078(.a(\b[15] ), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  aoai13aa1n06x5               g080(.a(new_n175), .b(new_n156), .c(new_n152), .d(new_n157), .o1(new_n176));
  nanp02aa1n03x5               g081(.a(new_n176), .b(new_n164), .o1(new_n177));
  oaib12aa1n06x5               g082(.a(new_n177), .b(new_n174), .c(\a[16] ), .out0(new_n178));
  oaib12aa1n18x5               g083(.a(new_n178), .b(\b[15] ), .c(new_n173), .out0(new_n179));
  aoi012aa1d18x5               g084(.a(new_n179), .b(new_n171), .c(new_n172), .o1(new_n180));
  oai012aa1d24x5               g085(.a(new_n180), .b(new_n119), .c(new_n170), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g087(.a(\a[18] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  xroi22aa1d06x4               g092(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n188));
  nor042aa1n02x5               g093(.a(\b[17] ), .b(\a[18] ), .o1(new_n189));
  aoi112aa1n09x5               g094(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n190));
  nor042aa1n06x5               g095(.a(new_n190), .b(new_n189), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  nor002aa1d32x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nand02aa1n04x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n03x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n192), .c(new_n181), .d(new_n188), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n192), .c(new_n181), .d(new_n188), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n12x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand02aa1n08x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n03x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nona22aa1n03x5               g107(.a(new_n196), .b(new_n202), .c(new_n193), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n193), .o1(new_n204));
  aobi12aa1n06x5               g109(.a(new_n202), .b(new_n196), .c(new_n204), .out0(new_n205));
  norb02aa1n03x4               g110(.a(new_n203), .b(new_n205), .out0(\s[20] ));
  nona23aa1n09x5               g111(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n207));
  inv040aa1n03x5               g112(.a(new_n207), .o1(new_n208));
  nand02aa1d04x5               g113(.a(new_n188), .b(new_n208), .o1(new_n209));
  inv040aa1n06x5               g114(.a(new_n209), .o1(new_n210));
  tech160nm_fioai012aa1n05x5   g115(.a(new_n201), .b(new_n200), .c(new_n193), .o1(new_n211));
  oai012aa1n12x5               g116(.a(new_n211), .b(new_n207), .c(new_n191), .o1(new_n212));
  xorc02aa1n12x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n212), .c(new_n181), .d(new_n210), .o1(new_n214));
  aoi112aa1n02x5               g119(.a(new_n213), .b(new_n212), .c(new_n181), .d(new_n210), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n214), .b(new_n215), .out0(\s[21] ));
  nor042aa1n03x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n12x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  nona22aa1n02x5               g123(.a(new_n214), .b(new_n218), .c(new_n217), .out0(new_n219));
  inv040aa1n03x5               g124(.a(new_n217), .o1(new_n220));
  aobi12aa1n02x7               g125(.a(new_n218), .b(new_n214), .c(new_n220), .out0(new_n221));
  norb02aa1n03x4               g126(.a(new_n219), .b(new_n221), .out0(\s[22] ));
  nand02aa1d04x5               g127(.a(new_n218), .b(new_n213), .o1(new_n223));
  nano22aa1n02x4               g128(.a(new_n223), .b(new_n188), .c(new_n208), .out0(new_n224));
  oai112aa1n03x5               g129(.a(new_n195), .b(new_n202), .c(new_n190), .d(new_n189), .o1(new_n225));
  oaoi03aa1n09x5               g130(.a(\a[22] ), .b(\b[21] ), .c(new_n220), .o1(new_n226));
  inv000aa1n02x5               g131(.a(new_n226), .o1(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n223), .c(new_n225), .d(new_n211), .o1(new_n228));
  xorc02aa1n12x5               g133(.a(\a[23] ), .b(\b[22] ), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n228), .c(new_n181), .d(new_n224), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n229), .b(new_n228), .c(new_n181), .d(new_n224), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n230), .b(new_n231), .out0(\s[23] ));
  nor042aa1n03x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  tech160nm_fixorc02aa1n02p5x5 g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  nona22aa1n02x5               g139(.a(new_n230), .b(new_n234), .c(new_n233), .out0(new_n235));
  inv020aa1n02x5               g140(.a(new_n233), .o1(new_n236));
  aobi12aa1n02x7               g141(.a(new_n234), .b(new_n230), .c(new_n236), .out0(new_n237));
  norb02aa1n03x4               g142(.a(new_n235), .b(new_n237), .out0(\s[24] ));
  nanp02aa1n02x5               g143(.a(new_n234), .b(new_n229), .o1(new_n239));
  nor003aa1n03x5               g144(.a(new_n209), .b(new_n223), .c(new_n239), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n223), .o1(new_n241));
  inv030aa1n03x5               g146(.a(new_n239), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n226), .c(new_n212), .d(new_n241), .o1(new_n243));
  oaoi03aa1n12x5               g148(.a(\a[24] ), .b(\b[23] ), .c(new_n236), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  nanp02aa1n02x5               g150(.a(new_n243), .b(new_n245), .o1(new_n246));
  xorc02aa1n12x5               g151(.a(\a[25] ), .b(\b[24] ), .out0(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n246), .c(new_n181), .d(new_n240), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n247), .b(new_n246), .c(new_n181), .d(new_n240), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n248), .b(new_n249), .out0(\s[25] ));
  nor042aa1n03x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  xorc02aa1n12x5               g156(.a(\a[26] ), .b(\b[25] ), .out0(new_n252));
  nona22aa1n03x5               g157(.a(new_n248), .b(new_n252), .c(new_n251), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n251), .o1(new_n254));
  aobi12aa1n06x5               g159(.a(new_n252), .b(new_n248), .c(new_n254), .out0(new_n255));
  norb02aa1n03x4               g160(.a(new_n253), .b(new_n255), .out0(\s[26] ));
  nor002aa1n02x5               g161(.a(new_n144), .b(new_n169), .o1(new_n257));
  oaoi03aa1n02x5               g162(.a(new_n173), .b(new_n174), .c(new_n177), .o1(new_n258));
  oai012aa1n02x5               g163(.a(new_n258), .b(new_n149), .c(new_n169), .o1(new_n259));
  nand02aa1d04x5               g164(.a(new_n252), .b(new_n247), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  nano32aa1n03x7               g166(.a(new_n209), .b(new_n261), .c(new_n241), .d(new_n242), .out0(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n259), .c(new_n125), .d(new_n257), .o1(new_n263));
  aoai13aa1n04x5               g168(.a(new_n261), .b(new_n244), .c(new_n228), .d(new_n242), .o1(new_n264));
  oao003aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .c(new_n254), .carry(new_n265));
  xorc02aa1n12x5               g170(.a(\a[27] ), .b(\b[26] ), .out0(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  aoi013aa1n06x4               g172(.a(new_n267), .b(new_n263), .c(new_n264), .d(new_n265), .o1(new_n268));
  nona32aa1n03x5               g173(.a(new_n210), .b(new_n260), .c(new_n239), .d(new_n223), .out0(new_n269));
  oaoi13aa1n09x5               g174(.a(new_n269), .b(new_n180), .c(new_n119), .d(new_n170), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n265), .b(new_n260), .c(new_n243), .d(new_n245), .o1(new_n271));
  norp03aa1n02x5               g176(.a(new_n271), .b(new_n270), .c(new_n266), .o1(new_n272));
  norp02aa1n02x5               g177(.a(new_n268), .b(new_n272), .o1(\s[27] ));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  inv040aa1n03x5               g179(.a(new_n274), .o1(new_n275));
  tech160nm_fixnrc02aa1n04x5   g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  nano22aa1n03x5               g181(.a(new_n268), .b(new_n275), .c(new_n276), .out0(new_n277));
  oaih12aa1n02x5               g182(.a(new_n266), .b(new_n271), .c(new_n270), .o1(new_n278));
  aoi012aa1n03x5               g183(.a(new_n276), .b(new_n278), .c(new_n275), .o1(new_n279));
  nor002aa1n02x5               g184(.a(new_n279), .b(new_n277), .o1(\s[28] ));
  xnrc02aa1n02x5               g185(.a(\b[28] ), .b(\a[29] ), .out0(new_n281));
  norb02aa1n02x5               g186(.a(new_n266), .b(new_n276), .out0(new_n282));
  oaih12aa1n02x5               g187(.a(new_n282), .b(new_n271), .c(new_n270), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n284));
  aoi012aa1n02x7               g189(.a(new_n281), .b(new_n283), .c(new_n284), .o1(new_n285));
  inv000aa1n02x5               g190(.a(new_n282), .o1(new_n286));
  aoi013aa1n03x5               g191(.a(new_n286), .b(new_n263), .c(new_n264), .d(new_n265), .o1(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n281), .c(new_n284), .out0(new_n288));
  nor002aa1n02x5               g193(.a(new_n285), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g195(.a(\b[29] ), .b(\a[30] ), .out0(new_n291));
  norb03aa1n12x5               g196(.a(new_n266), .b(new_n281), .c(new_n276), .out0(new_n292));
  oaih12aa1n02x5               g197(.a(new_n292), .b(new_n271), .c(new_n270), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n294));
  tech160nm_fiaoi012aa1n02p5x5 g199(.a(new_n291), .b(new_n293), .c(new_n294), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n292), .o1(new_n296));
  aoi013aa1n03x5               g201(.a(new_n296), .b(new_n263), .c(new_n264), .d(new_n265), .o1(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n291), .c(new_n294), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n295), .b(new_n298), .o1(\s[30] ));
  norb02aa1n02x5               g204(.a(new_n292), .b(new_n291), .out0(new_n300));
  inv000aa1n02x5               g205(.a(new_n300), .o1(new_n301));
  aoi013aa1n03x5               g206(.a(new_n301), .b(new_n263), .c(new_n264), .d(new_n265), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  nano22aa1n03x5               g209(.a(new_n302), .b(new_n303), .c(new_n304), .out0(new_n305));
  oaih12aa1n02x5               g210(.a(new_n300), .b(new_n271), .c(new_n270), .o1(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n304), .b(new_n306), .c(new_n303), .o1(new_n307));
  norp02aa1n03x5               g212(.a(new_n307), .b(new_n305), .o1(\s[31] ));
  inv000aa1d42x5               g213(.a(\a[3] ), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n108), .b(\b[2] ), .c(new_n309), .out0(\s[3] ));
  norp02aa1n02x5               g215(.a(new_n104), .b(new_n108), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[4] ), .b(\b[3] ), .out0(new_n312));
  aoib12aa1n02x5               g217(.a(new_n312), .b(new_n309), .c(\b[2] ), .out0(new_n313));
  aboi22aa1n03x5               g218(.a(new_n311), .b(new_n313), .c(new_n109), .d(new_n312), .out0(\s[4] ));
  xnbna2aa1n03x5               g219(.a(new_n113), .b(new_n109), .c(new_n110), .out0(\s[5] ));
  nanp02aa1n02x5               g220(.a(new_n109), .b(new_n110), .o1(new_n316));
  oaoi03aa1n02x5               g221(.a(\a[5] ), .b(\b[4] ), .c(new_n316), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g223(.a(new_n116), .b(new_n109), .c(new_n114), .o(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g225(.a(new_n99), .b(new_n319), .c(new_n100), .o1(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g227(.a(new_n126), .b(new_n123), .c(new_n124), .out0(\s[9] ));
endmodule


