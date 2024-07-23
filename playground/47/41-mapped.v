// Benchmark "adder" written by ABC on Thu Jul 18 12:28:04 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n308, new_n310,
    new_n311, new_n314, new_n315, new_n316, new_n319, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n05x5   g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n24x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor042aa1n02x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nanp02aa1n06x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  norb02aa1n02x7               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  nor042aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nanp02aa1n24x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  norb02aa1n02x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  tech160nm_fixnrc02aa1n04x5   g020(.a(\b[6] ), .b(\a[7] ), .out0(new_n116));
  xnrc02aa1n12x5               g021(.a(\b[7] ), .b(\a[8] ), .out0(new_n117));
  nano23aa1n06x5               g022(.a(new_n117), .b(new_n116), .c(new_n115), .d(new_n112), .out0(new_n118));
  inv000aa1d42x5               g023(.a(\a[7] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[6] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  and002aa1n02x5               g026(.a(\b[7] ), .b(\a[8] ), .o(new_n122));
  orn002aa1n02x5               g027(.a(\a[8] ), .b(\b[7] ), .o(new_n123));
  oai022aa1n02x7               g028(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  oai112aa1n02x5               g029(.a(new_n124), .b(new_n114), .c(new_n120), .d(new_n119), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n123), .b(new_n122), .c(new_n125), .d(new_n121), .o1(new_n126));
  xorc02aa1n02x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n06x5               g032(.a(new_n127), .b(new_n126), .c(new_n118), .d(new_n109), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n97), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  oai022aa1n06x5               g034(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n130));
  nanb02aa1n03x5               g035(.a(new_n130), .b(new_n128), .out0(new_n131));
  aoi022aa1d24x5               g036(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n132));
  oai112aa1n03x5               g037(.a(new_n131), .b(new_n132), .c(\b[10] ), .d(\a[11] ), .o1(new_n133));
  xorc02aa1n02x5               g038(.a(\a[11] ), .b(\b[10] ), .out0(new_n134));
  aboi22aa1n03x5               g039(.a(new_n130), .b(new_n128), .c(\b[9] ), .d(\a[10] ), .out0(new_n135));
  oa0012aa1n02x5               g040(.a(new_n133), .b(new_n135), .c(new_n134), .o(\s[11] ));
  nor042aa1n03x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nor042aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  and002aa1n12x5               g043(.a(\b[11] ), .b(\a[12] ), .o(new_n139));
  obai22aa1n02x7               g044(.a(new_n133), .b(new_n137), .c(new_n138), .d(new_n139), .out0(new_n140));
  nor043aa1n06x5               g045(.a(new_n139), .b(new_n138), .c(new_n137), .o1(new_n141));
  aob012aa1n03x5               g046(.a(new_n140), .b(new_n133), .c(new_n141), .out0(\s[12] ));
  nanp02aa1n02x5               g047(.a(\b[8] ), .b(\a[9] ), .o1(new_n143));
  nano22aa1n02x4               g048(.a(new_n137), .b(new_n132), .c(new_n143), .out0(new_n144));
  nand02aa1d04x5               g049(.a(new_n130), .b(new_n132), .o1(new_n145));
  and003aa1n02x5               g050(.a(new_n144), .b(new_n141), .c(new_n145), .o(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n126), .c(new_n118), .d(new_n109), .o1(new_n147));
  aoi012aa1n12x5               g052(.a(new_n139), .b(new_n141), .c(new_n145), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  nor042aa1d18x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nand42aa1d28x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  norb02aa1n02x5               g056(.a(new_n151), .b(new_n150), .out0(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n152), .b(new_n147), .c(new_n149), .out0(\s[13] ));
  inv000aa1d42x5               g058(.a(new_n150), .o1(new_n154));
  aob012aa1n02x5               g059(.a(new_n152), .b(new_n147), .c(new_n149), .out0(new_n155));
  nor042aa1n04x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand42aa1d28x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  oai022aa1n02x5               g063(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n159));
  nanb03aa1n02x5               g064(.a(new_n159), .b(new_n155), .c(new_n157), .out0(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n158), .c(new_n154), .d(new_n155), .o1(\s[14] ));
  nano23aa1d15x5               g066(.a(new_n150), .b(new_n156), .c(new_n157), .d(new_n151), .out0(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  oai012aa1n02x5               g068(.a(new_n157), .b(new_n156), .c(new_n150), .o1(new_n164));
  aoai13aa1n04x5               g069(.a(new_n164), .b(new_n163), .c(new_n147), .d(new_n149), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand42aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanb02aa1n03x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  norb02aa1n02x5               g074(.a(new_n165), .b(new_n169), .out0(new_n170));
  xnrc02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .out0(new_n171));
  aoai13aa1n02x5               g076(.a(new_n171), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n172));
  inv000aa1n02x5               g077(.a(new_n167), .o1(new_n173));
  orn002aa1n02x5               g078(.a(\a[16] ), .b(\b[15] ), .o(new_n174));
  and002aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o(new_n175));
  nanb03aa1n02x5               g080(.a(new_n175), .b(new_n173), .c(new_n174), .out0(new_n176));
  oai012aa1n02x5               g081(.a(new_n172), .b(new_n170), .c(new_n176), .o1(\s[16] ));
  nona22aa1n03x5               g082(.a(new_n162), .b(new_n171), .c(new_n169), .out0(new_n178));
  nano32aa1n03x7               g083(.a(new_n178), .b(new_n144), .c(new_n145), .d(new_n141), .out0(new_n179));
  aoai13aa1n09x5               g084(.a(new_n179), .b(new_n126), .c(new_n109), .d(new_n118), .o1(new_n180));
  nanp03aa1n02x5               g085(.a(new_n159), .b(new_n157), .c(new_n168), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n174), .b(new_n175), .c(new_n181), .d(new_n173), .o1(new_n182));
  aoib12aa1n12x5               g087(.a(new_n182), .b(new_n148), .c(new_n178), .out0(new_n183));
  xorc02aa1n12x5               g088(.a(\a[17] ), .b(\b[16] ), .out0(new_n184));
  xnbna2aa1n03x5               g089(.a(new_n184), .b(new_n180), .c(new_n183), .out0(\s[17] ));
  aobi12aa1n02x5               g090(.a(new_n184), .b(new_n180), .c(new_n183), .out0(new_n186));
  xorc02aa1n03x5               g091(.a(\a[18] ), .b(\b[17] ), .out0(new_n187));
  nanp02aa1n12x5               g092(.a(new_n180), .b(new_n183), .o1(new_n188));
  orn002aa1n02x5               g093(.a(\a[17] ), .b(\b[16] ), .o(new_n189));
  aobi12aa1n02x5               g094(.a(new_n189), .b(new_n188), .c(new_n184), .out0(new_n190));
  oaih22aa1n04x5               g095(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n191));
  tech160nm_fiao0012aa1n02p5x5 g096(.a(new_n191), .b(\a[18] ), .c(\b[17] ), .o(new_n192));
  oai022aa1n02x5               g097(.a(new_n190), .b(new_n187), .c(new_n192), .d(new_n186), .o1(\s[18] ));
  nanp02aa1n02x5               g098(.a(new_n187), .b(new_n184), .o1(new_n194));
  aob012aa1n02x5               g099(.a(new_n191), .b(\b[17] ), .c(\a[18] ), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n194), .c(new_n180), .d(new_n183), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand22aa1n03x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nor022aa1n08x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nand22aa1n03x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  aoai13aa1n02x5               g108(.a(new_n203), .b(new_n199), .c(new_n196), .d(new_n200), .o1(new_n204));
  aoi112aa1n03x5               g109(.a(new_n199), .b(new_n203), .c(new_n196), .d(new_n200), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(\s[20] ));
  nano23aa1n06x5               g111(.a(new_n199), .b(new_n201), .c(new_n202), .d(new_n200), .out0(new_n207));
  nand23aa1n06x5               g112(.a(new_n207), .b(new_n184), .c(new_n187), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  nona23aa1n02x4               g114(.a(new_n202), .b(new_n200), .c(new_n199), .d(new_n201), .out0(new_n210));
  tech160nm_fioai012aa1n04x5   g115(.a(new_n202), .b(new_n201), .c(new_n199), .o1(new_n211));
  tech160nm_fioai012aa1n03p5x5 g116(.a(new_n211), .b(new_n210), .c(new_n195), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n212), .c(new_n188), .d(new_n209), .o1(new_n214));
  aoi112aa1n02x5               g119(.a(new_n213), .b(new_n212), .c(new_n188), .d(new_n209), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n214), .b(new_n215), .out0(\s[21] ));
  nor042aa1d18x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  inv040aa1n03x5               g122(.a(new_n217), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  inv000aa1d42x5               g124(.a(\a[22] ), .o1(new_n220));
  inv000aa1d42x5               g125(.a(\b[21] ), .o1(new_n221));
  aoi012aa1n02x5               g126(.a(new_n217), .b(new_n220), .c(new_n221), .o1(new_n222));
  oai112aa1n03x5               g127(.a(new_n214), .b(new_n222), .c(new_n221), .d(new_n220), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n223), .b(new_n219), .c(new_n218), .d(new_n214), .o1(\s[22] ));
  nano32aa1n02x4               g129(.a(new_n194), .b(new_n219), .c(new_n207), .d(new_n213), .out0(new_n225));
  oaoi03aa1n02x5               g130(.a(\a[18] ), .b(\b[17] ), .c(new_n189), .o1(new_n226));
  inv000aa1n02x5               g131(.a(new_n211), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\a[21] ), .o1(new_n228));
  xroi22aa1d06x4               g133(.a(new_n228), .b(\b[20] ), .c(new_n220), .d(\b[21] ), .out0(new_n229));
  aoai13aa1n03x5               g134(.a(new_n229), .b(new_n227), .c(new_n207), .d(new_n226), .o1(new_n230));
  oaoi03aa1n06x5               g135(.a(\a[22] ), .b(\b[21] ), .c(new_n218), .o1(new_n231));
  inv040aa1d30x5               g136(.a(new_n231), .o1(new_n232));
  nanp02aa1n02x5               g137(.a(new_n230), .b(new_n232), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[22] ), .b(\a[23] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoai13aa1n06x5               g140(.a(new_n235), .b(new_n233), .c(new_n188), .d(new_n225), .o1(new_n236));
  aoi112aa1n02x5               g141(.a(new_n235), .b(new_n233), .c(new_n188), .d(new_n225), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n236), .b(new_n237), .out0(\s[23] ));
  nor042aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  inv000aa1d42x5               g146(.a(\a[24] ), .o1(new_n242));
  inv000aa1d42x5               g147(.a(\b[23] ), .o1(new_n243));
  aoi012aa1n02x5               g148(.a(new_n239), .b(new_n242), .c(new_n243), .o1(new_n244));
  oai112aa1n03x5               g149(.a(new_n236), .b(new_n244), .c(new_n243), .d(new_n242), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n241), .c(new_n240), .d(new_n236), .o1(\s[24] ));
  norb02aa1n02x5               g151(.a(new_n241), .b(new_n234), .out0(new_n247));
  nano22aa1n02x5               g152(.a(new_n208), .b(new_n247), .c(new_n229), .out0(new_n248));
  inv000aa1n02x5               g153(.a(new_n247), .o1(new_n249));
  oaoi03aa1n02x5               g154(.a(new_n242), .b(new_n243), .c(new_n239), .o1(new_n250));
  aoai13aa1n04x5               g155(.a(new_n250), .b(new_n249), .c(new_n230), .d(new_n232), .o1(new_n251));
  xorc02aa1n12x5               g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n251), .c(new_n188), .d(new_n248), .o1(new_n253));
  aoi112aa1n02x5               g158(.a(new_n252), .b(new_n251), .c(new_n188), .d(new_n248), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n253), .b(new_n254), .out0(\s[25] ));
  nor042aa1n03x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  xorc02aa1n02x5               g162(.a(\a[26] ), .b(\b[25] ), .out0(new_n258));
  nanp02aa1n02x5               g163(.a(\b[25] ), .b(\a[26] ), .o1(new_n259));
  inv000aa1d42x5               g164(.a(\a[26] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(\b[25] ), .o1(new_n261));
  aoi012aa1n02x5               g166(.a(new_n256), .b(new_n260), .c(new_n261), .o1(new_n262));
  nanp03aa1n03x5               g167(.a(new_n253), .b(new_n259), .c(new_n262), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n263), .b(new_n258), .c(new_n257), .d(new_n253), .o1(\s[26] ));
  nanp02aa1n04x5               g169(.a(new_n258), .b(new_n252), .o1(new_n265));
  nano23aa1n06x5               g170(.a(new_n208), .b(new_n265), .c(new_n247), .d(new_n229), .out0(new_n266));
  nanp02aa1n03x5               g171(.a(new_n188), .b(new_n266), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n262), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n265), .o1(new_n269));
  aoi022aa1n02x7               g174(.a(new_n251), .b(new_n269), .c(new_n259), .d(new_n268), .o1(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnbna2aa1n03x5               g176(.a(new_n271), .b(new_n267), .c(new_n270), .out0(\s[27] ));
  xorc02aa1n02x5               g177(.a(\a[28] ), .b(\b[27] ), .out0(new_n273));
  aobi12aa1n06x5               g178(.a(new_n266), .b(new_n180), .c(new_n183), .out0(new_n274));
  aoai13aa1n04x5               g179(.a(new_n247), .b(new_n231), .c(new_n212), .d(new_n229), .o1(new_n275));
  oaoi03aa1n02x5               g180(.a(new_n260), .b(new_n261), .c(new_n256), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n265), .c(new_n275), .d(new_n250), .o1(new_n277));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  oaoi13aa1n03x5               g183(.a(new_n278), .b(new_n271), .c(new_n277), .d(new_n274), .o1(new_n279));
  oaih12aa1n02x5               g184(.a(new_n271), .b(new_n277), .c(new_n274), .o1(new_n280));
  oai112aa1n03x5               g185(.a(new_n280), .b(new_n273), .c(\b[26] ), .d(\a[27] ), .o1(new_n281));
  oaih12aa1n02x5               g186(.a(new_n281), .b(new_n279), .c(new_n273), .o1(\s[28] ));
  and002aa1n02x5               g187(.a(new_n273), .b(new_n271), .o(new_n283));
  oaih12aa1n02x5               g188(.a(new_n283), .b(new_n277), .c(new_n274), .o1(new_n284));
  orn002aa1n03x5               g189(.a(\a[27] ), .b(\b[26] ), .o(new_n285));
  oao003aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .c(new_n285), .carry(new_n286));
  nanp02aa1n03x5               g191(.a(new_n284), .b(new_n286), .o1(new_n287));
  xorc02aa1n02x5               g192(.a(\a[29] ), .b(\b[28] ), .out0(new_n288));
  norb02aa1n02x5               g193(.a(new_n286), .b(new_n288), .out0(new_n289));
  aoi022aa1n02x7               g194(.a(new_n287), .b(new_n288), .c(new_n284), .d(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g196(.a(new_n271), .b(new_n288), .c(new_n273), .o(new_n292));
  oaih12aa1n02x5               g197(.a(new_n292), .b(new_n277), .c(new_n274), .o1(new_n293));
  oaoi03aa1n09x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  nanp02aa1n03x5               g200(.a(new_n293), .b(new_n295), .o1(new_n296));
  xorc02aa1n02x5               g201(.a(\a[30] ), .b(\b[29] ), .out0(new_n297));
  norp02aa1n02x5               g202(.a(new_n294), .b(new_n297), .o1(new_n298));
  aoi022aa1n02x7               g203(.a(new_n296), .b(new_n297), .c(new_n293), .d(new_n298), .o1(\s[30] ));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  and003aa1n02x5               g205(.a(new_n283), .b(new_n297), .c(new_n288), .o(new_n301));
  oaih12aa1n02x5               g206(.a(new_n301), .b(new_n277), .c(new_n274), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n295), .carry(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n300), .b(new_n302), .c(new_n303), .o1(new_n304));
  aobi12aa1n03x5               g209(.a(new_n301), .b(new_n267), .c(new_n270), .out0(new_n305));
  nano22aa1n03x5               g210(.a(new_n305), .b(new_n300), .c(new_n303), .out0(new_n306));
  norp02aa1n03x5               g211(.a(new_n304), .b(new_n306), .o1(\s[31] ));
  inv000aa1d42x5               g212(.a(new_n105), .o1(new_n308));
  xnbna2aa1n03x5               g213(.a(new_n102), .b(new_n106), .c(new_n308), .out0(\s[3] ));
  norb02aa1n02x5               g214(.a(new_n104), .b(new_n103), .out0(new_n310));
  nanb03aa1n02x5               g215(.a(new_n102), .b(new_n106), .c(new_n308), .out0(new_n311));
  xnbna2aa1n03x5               g216(.a(new_n310), .b(new_n311), .c(new_n308), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g218(.a(new_n110), .b(new_n109), .c(new_n111), .o1(new_n314));
  norb03aa1n02x5               g219(.a(new_n114), .b(new_n110), .c(new_n113), .out0(new_n315));
  aob012aa1n02x5               g220(.a(new_n315), .b(new_n109), .c(new_n112), .out0(new_n316));
  oai012aa1n02x5               g221(.a(new_n316), .b(new_n314), .c(new_n115), .o1(\s[6] ));
  xnbna2aa1n03x5               g222(.a(new_n116), .b(new_n316), .c(new_n114), .out0(\s[7] ));
  nanb03aa1n02x5               g223(.a(new_n116), .b(new_n316), .c(new_n114), .out0(new_n319));
  xobna2aa1n03x5               g224(.a(new_n117), .b(new_n319), .c(new_n121), .out0(\s[8] ));
  aoi112aa1n02x5               g225(.a(new_n126), .b(new_n127), .c(new_n118), .d(new_n109), .o1(new_n321));
  norb02aa1n02x5               g226(.a(new_n128), .b(new_n321), .out0(\s[9] ));
endmodule


