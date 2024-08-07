// Benchmark "adder" written by ABC on Thu Jul 18 12:25:35 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n315, new_n317,
    new_n318, new_n321, new_n322, new_n323, new_n324, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(\b[8] ), .b(new_n98), .out0(new_n99));
  nor002aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nand22aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi012aa1n12x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  norp02aa1n12x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand22aa1n04x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n09x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  tech160nm_fioai012aa1n03p5x5 g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oai012aa1n18x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  tech160nm_finand02aa1n03p5x5 g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nor022aa1n08x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand02aa1d10x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nona23aa1d18x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  xnrc02aa1n12x5               g020(.a(\b[6] ), .b(\a[7] ), .out0(new_n116));
  tech160nm_fixnrc02aa1n04x5   g021(.a(\b[7] ), .b(\a[8] ), .out0(new_n117));
  norp03aa1d24x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  inv040aa1d32x5               g023(.a(\a[7] ), .o1(new_n119));
  inv030aa1d32x5               g024(.a(\b[6] ), .o1(new_n120));
  nand42aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  and002aa1n02x5               g026(.a(\b[7] ), .b(\a[8] ), .o(new_n122));
  orn002aa1n02x5               g027(.a(\a[8] ), .b(\b[7] ), .o(new_n123));
  oai022aa1d18x5               g028(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  oai112aa1n06x5               g029(.a(new_n124), .b(new_n114), .c(new_n120), .d(new_n119), .o1(new_n125));
  aoai13aa1n09x5               g030(.a(new_n123), .b(new_n122), .c(new_n125), .d(new_n121), .o1(new_n126));
  xorc02aa1n02x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n06x5               g032(.a(new_n127), .b(new_n126), .c(new_n110), .d(new_n118), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n97), .b(new_n128), .c(new_n99), .out0(\s[10] ));
  oai022aa1d24x5               g034(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n130));
  nanb02aa1n02x5               g035(.a(new_n130), .b(new_n128), .out0(new_n131));
  nanp02aa1n12x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand02aa1n10x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanb03aa1d24x5               g039(.a(new_n134), .b(new_n132), .c(new_n133), .out0(new_n135));
  inv000aa1d42x5               g040(.a(new_n135), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n134), .b(new_n133), .out0(new_n137));
  nanp02aa1n02x5               g042(.a(new_n131), .b(new_n132), .o1(new_n138));
  aoi022aa1n02x5               g043(.a(new_n138), .b(new_n137), .c(new_n131), .d(new_n136), .o1(\s[11] ));
  nanp02aa1n03x5               g044(.a(new_n131), .b(new_n136), .o1(new_n140));
  nor042aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  and002aa1n24x5               g046(.a(\b[11] ), .b(\a[12] ), .o(new_n142));
  obai22aa1n02x7               g047(.a(new_n140), .b(new_n134), .c(new_n141), .d(new_n142), .out0(new_n143));
  nor043aa1d12x5               g048(.a(new_n142), .b(new_n141), .c(new_n134), .o1(new_n144));
  inv030aa1n04x5               g049(.a(new_n144), .o1(new_n145));
  oaib12aa1n02x5               g050(.a(new_n143), .b(new_n145), .c(new_n140), .out0(\s[12] ));
  nanp02aa1n02x5               g051(.a(\b[8] ), .b(\a[9] ), .o1(new_n147));
  oai012aa1d24x5               g052(.a(new_n130), .b(\b[11] ), .c(\a[12] ), .o1(new_n148));
  nano23aa1n09x5               g053(.a(new_n145), .b(new_n135), .c(new_n148), .d(new_n147), .out0(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n126), .c(new_n110), .d(new_n118), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n142), .o1(new_n151));
  oai012aa1n12x5               g056(.a(new_n144), .b(new_n135), .c(new_n148), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(new_n152), .b(new_n151), .o1(new_n153));
  nand42aa1n02x5               g058(.a(new_n150), .b(new_n153), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  nanp02aa1n06x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nand03aa1n02x5               g063(.a(new_n154), .b(new_n157), .c(new_n158), .o1(new_n159));
  nor022aa1n12x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n06x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  oaih22aa1d12x5               g067(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n163));
  nanb03aa1n02x5               g068(.a(new_n163), .b(new_n159), .c(new_n161), .out0(new_n164));
  aoai13aa1n03x5               g069(.a(new_n164), .b(new_n162), .c(new_n157), .d(new_n159), .o1(\s[14] ));
  nona23aa1n09x5               g070(.a(new_n161), .b(new_n158), .c(new_n156), .d(new_n160), .out0(new_n166));
  oai012aa1n02x5               g071(.a(new_n161), .b(new_n160), .c(new_n156), .o1(new_n167));
  aoai13aa1n06x5               g072(.a(new_n167), .b(new_n166), .c(new_n150), .d(new_n153), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1d18x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nand02aa1n03x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanb02aa1n06x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  norb02aa1n03x5               g077(.a(new_n168), .b(new_n172), .out0(new_n173));
  xnrc02aa1n12x5               g078(.a(\b[15] ), .b(\a[16] ), .out0(new_n174));
  aoai13aa1n03x5               g079(.a(new_n174), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n170), .o1(new_n176));
  orn002aa1n02x5               g081(.a(\a[16] ), .b(\b[15] ), .o(new_n177));
  and002aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o(new_n178));
  nanb03aa1n02x5               g083(.a(new_n178), .b(new_n176), .c(new_n177), .out0(new_n179));
  oai012aa1n02x5               g084(.a(new_n175), .b(new_n173), .c(new_n179), .o1(\s[16] ));
  aoi012aa1d24x5               g085(.a(new_n126), .b(new_n110), .c(new_n118), .o1(new_n181));
  nor043aa1n06x5               g086(.a(new_n166), .b(new_n172), .c(new_n174), .o1(new_n182));
  nand22aa1n06x5               g087(.a(new_n149), .b(new_n182), .o1(new_n183));
  nand43aa1n02x5               g088(.a(new_n163), .b(new_n161), .c(new_n171), .o1(new_n184));
  aoai13aa1n06x5               g089(.a(new_n177), .b(new_n178), .c(new_n184), .d(new_n176), .o1(new_n185));
  aoi013aa1n09x5               g090(.a(new_n185), .b(new_n182), .c(new_n152), .d(new_n151), .o1(new_n186));
  oai012aa1d24x5               g091(.a(new_n186), .b(new_n181), .c(new_n183), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  tech160nm_fixnrc02aa1n04x5   g093(.a(\b[16] ), .b(\a[17] ), .out0(new_n189));
  oaoi13aa1n02x5               g094(.a(new_n189), .b(new_n186), .c(new_n181), .d(new_n183), .o1(new_n190));
  tech160nm_fixorc02aa1n03p5x5 g095(.a(\a[18] ), .b(\b[17] ), .out0(new_n191));
  inv000aa1d42x5               g096(.a(\a[17] ), .o1(new_n192));
  inv000aa1d42x5               g097(.a(\b[16] ), .o1(new_n193));
  oaoi03aa1n02x5               g098(.a(new_n192), .b(new_n193), .c(new_n187), .o1(new_n194));
  oai022aa1d24x5               g099(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n195));
  tech160nm_fiao0012aa1n02p5x5 g100(.a(new_n195), .b(\a[18] ), .c(\b[17] ), .o(new_n196));
  oai022aa1n02x5               g101(.a(new_n194), .b(new_n191), .c(new_n196), .d(new_n190), .o1(\s[18] ));
  inv000aa1d42x5               g102(.a(\b[17] ), .o1(new_n198));
  xroi22aa1d04x5               g103(.a(new_n198), .b(\a[18] ), .c(new_n193), .d(\a[17] ), .out0(new_n199));
  nand22aa1n09x5               g104(.a(new_n187), .b(new_n199), .o1(new_n200));
  oaib12aa1n18x5               g105(.a(new_n195), .b(new_n198), .c(\a[18] ), .out0(new_n201));
  nor002aa1d32x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  tech160nm_finand02aa1n05x5   g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n200), .c(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n02x5               g112(.a(new_n200), .b(new_n201), .o1(new_n208));
  nor002aa1d32x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand02aa1n06x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  aoai13aa1n03x5               g116(.a(new_n211), .b(new_n202), .c(new_n208), .d(new_n205), .o1(new_n212));
  norb03aa1n02x5               g117(.a(new_n210), .b(new_n202), .c(new_n209), .out0(new_n213));
  aoai13aa1n02x7               g118(.a(new_n213), .b(new_n204), .c(new_n200), .d(new_n201), .o1(new_n214));
  nanp02aa1n03x5               g119(.a(new_n212), .b(new_n214), .o1(\s[20] ));
  nona23aa1d18x5               g120(.a(new_n210), .b(new_n203), .c(new_n202), .d(new_n209), .out0(new_n216));
  norb03aa1n12x5               g121(.a(new_n191), .b(new_n216), .c(new_n189), .out0(new_n217));
  oai012aa1n02x5               g122(.a(new_n210), .b(new_n209), .c(new_n202), .o1(new_n218));
  oai012aa1n06x5               g123(.a(new_n218), .b(new_n216), .c(new_n201), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[21] ), .b(\b[20] ), .out0(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n219), .c(new_n187), .d(new_n217), .o1(new_n221));
  aoi112aa1n02x5               g126(.a(new_n220), .b(new_n219), .c(new_n187), .d(new_n217), .o1(new_n222));
  norb02aa1n02x7               g127(.a(new_n221), .b(new_n222), .out0(\s[21] ));
  nor042aa1n09x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  inv000aa1n06x5               g129(.a(new_n224), .o1(new_n225));
  xorc02aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .out0(new_n226));
  nand42aa1d28x5               g131(.a(\b[21] ), .b(\a[22] ), .o1(new_n227));
  oai022aa1n02x5               g132(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n228));
  nanb03aa1n03x5               g133(.a(new_n228), .b(new_n221), .c(new_n227), .out0(new_n229));
  aoai13aa1n03x5               g134(.a(new_n229), .b(new_n226), .c(new_n221), .d(new_n225), .o1(\s[22] ));
  nand42aa1d28x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  nor042aa1n02x5               g136(.a(\b[21] ), .b(\a[22] ), .o1(new_n232));
  nano23aa1n06x5               g137(.a(new_n224), .b(new_n232), .c(new_n227), .d(new_n231), .out0(new_n233));
  nano23aa1n02x4               g138(.a(new_n189), .b(new_n216), .c(new_n233), .d(new_n191), .out0(new_n234));
  aoi022aa1n02x5               g139(.a(new_n219), .b(new_n233), .c(new_n227), .d(new_n228), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n235), .o1(new_n236));
  xorc02aa1n12x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n236), .c(new_n187), .d(new_n234), .o1(new_n238));
  aoi112aa1n02x5               g143(.a(new_n237), .b(new_n236), .c(new_n187), .d(new_n234), .o1(new_n239));
  norb02aa1n02x7               g144(.a(new_n238), .b(new_n239), .out0(\s[23] ));
  norp02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  xorc02aa1n12x5               g147(.a(\a[24] ), .b(\b[23] ), .out0(new_n243));
  nanp02aa1n02x5               g148(.a(\b[23] ), .b(\a[24] ), .o1(new_n244));
  oai022aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n245));
  nanb03aa1n03x5               g150(.a(new_n245), .b(new_n238), .c(new_n244), .out0(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n243), .c(new_n238), .d(new_n242), .o1(\s[24] ));
  nano23aa1n09x5               g152(.a(new_n202), .b(new_n209), .c(new_n210), .d(new_n203), .out0(new_n248));
  nand02aa1d06x5               g153(.a(new_n243), .b(new_n237), .o1(new_n249));
  nano32aa1n02x4               g154(.a(new_n249), .b(new_n199), .c(new_n248), .d(new_n233), .out0(new_n250));
  inv030aa1n02x5               g155(.a(new_n201), .o1(new_n251));
  inv000aa1n02x5               g156(.a(new_n218), .o1(new_n252));
  aoai13aa1n06x5               g157(.a(new_n233), .b(new_n252), .c(new_n248), .d(new_n251), .o1(new_n253));
  oaoi03aa1n06x5               g158(.a(\a[22] ), .b(\b[21] ), .c(new_n225), .o1(new_n254));
  inv000aa1n02x5               g159(.a(new_n254), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(new_n245), .b(new_n244), .o1(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n249), .c(new_n253), .d(new_n255), .o1(new_n257));
  xorc02aa1n12x5               g162(.a(\a[25] ), .b(\b[24] ), .out0(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n257), .c(new_n187), .d(new_n250), .o1(new_n259));
  aoi112aa1n02x5               g164(.a(new_n258), .b(new_n257), .c(new_n187), .d(new_n250), .o1(new_n260));
  norb02aa1n02x7               g165(.a(new_n259), .b(new_n260), .out0(\s[25] ));
  norp02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  tech160nm_fixorc02aa1n05x5   g168(.a(\a[26] ), .b(\b[25] ), .out0(new_n264));
  nanp02aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .o1(new_n265));
  oai022aa1n02x5               g170(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n266));
  nanb03aa1n03x5               g171(.a(new_n266), .b(new_n259), .c(new_n265), .out0(new_n267));
  aoai13aa1n03x5               g172(.a(new_n267), .b(new_n264), .c(new_n259), .d(new_n263), .o1(\s[26] ));
  inv000aa1n02x5               g173(.a(new_n233), .o1(new_n269));
  and002aa1n09x5               g174(.a(new_n264), .b(new_n258), .o(new_n270));
  nona23aa1n09x5               g175(.a(new_n217), .b(new_n270), .c(new_n249), .d(new_n269), .out0(new_n271));
  inv000aa1n02x5               g176(.a(new_n271), .o1(new_n272));
  nand02aa1d06x5               g177(.a(new_n187), .b(new_n272), .o1(new_n273));
  aoi022aa1n06x5               g178(.a(new_n257), .b(new_n270), .c(new_n265), .d(new_n266), .o1(new_n274));
  xorc02aa1n02x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnbna2aa1n06x5               g180(.a(new_n275), .b(new_n274), .c(new_n273), .out0(\s[27] ));
  aobi12aa1n06x5               g181(.a(new_n275), .b(new_n274), .c(new_n273), .out0(new_n277));
  xorc02aa1n12x5               g182(.a(\a[28] ), .b(\b[27] ), .out0(new_n278));
  oaoi13aa1n09x5               g183(.a(new_n271), .b(new_n186), .c(new_n181), .d(new_n183), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n249), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n280), .b(new_n254), .c(new_n219), .d(new_n233), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n270), .o1(new_n282));
  nanp02aa1n02x5               g187(.a(new_n266), .b(new_n265), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n282), .c(new_n281), .d(new_n256), .o1(new_n284));
  norp02aa1n02x5               g189(.a(\b[26] ), .b(\a[27] ), .o1(new_n285));
  oaoi13aa1n03x5               g190(.a(new_n285), .b(new_n275), .c(new_n284), .d(new_n279), .o1(new_n286));
  inv000aa1d42x5               g191(.a(\a[27] ), .o1(new_n287));
  oaib12aa1n06x5               g192(.a(new_n278), .b(\b[26] ), .c(new_n287), .out0(new_n288));
  oai022aa1n03x5               g193(.a(new_n286), .b(new_n278), .c(new_n288), .d(new_n277), .o1(\s[28] ));
  inv000aa1d42x5               g194(.a(\a[28] ), .o1(new_n290));
  xroi22aa1d04x5               g195(.a(new_n287), .b(\b[26] ), .c(new_n290), .d(\b[27] ), .out0(new_n291));
  oai012aa1n06x5               g196(.a(new_n291), .b(new_n284), .c(new_n279), .o1(new_n292));
  oaib12aa1n09x5               g197(.a(new_n288), .b(new_n290), .c(\b[27] ), .out0(new_n293));
  tech160nm_finand02aa1n03p5x5 g198(.a(new_n292), .b(new_n293), .o1(new_n294));
  xorc02aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .out0(new_n295));
  norb02aa1n02x5               g200(.a(new_n293), .b(new_n295), .out0(new_n296));
  aoi022aa1n02x7               g201(.a(new_n294), .b(new_n295), .c(new_n292), .d(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g203(.a(new_n275), .b(new_n295), .c(new_n278), .o(new_n299));
  tech160nm_fioai012aa1n04x5   g204(.a(new_n299), .b(new_n284), .c(new_n279), .o1(new_n300));
  oaoi03aa1n09x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .o1(new_n301));
  inv000aa1n02x5               g206(.a(new_n301), .o1(new_n302));
  nand42aa1n02x5               g207(.a(new_n300), .b(new_n302), .o1(new_n303));
  xorc02aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .out0(new_n304));
  norp02aa1n02x5               g209(.a(new_n301), .b(new_n304), .o1(new_n305));
  aoi022aa1n02x7               g210(.a(new_n303), .b(new_n304), .c(new_n300), .d(new_n305), .o1(\s[30] ));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  and003aa1n02x5               g212(.a(new_n291), .b(new_n304), .c(new_n295), .o(new_n308));
  oaih12aa1n02x5               g213(.a(new_n308), .b(new_n284), .c(new_n279), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n310));
  tech160nm_fiaoi012aa1n02p5x5 g215(.a(new_n307), .b(new_n309), .c(new_n310), .o1(new_n311));
  aobi12aa1n06x5               g216(.a(new_n308), .b(new_n274), .c(new_n273), .out0(new_n312));
  nano22aa1n03x5               g217(.a(new_n312), .b(new_n307), .c(new_n310), .out0(new_n313));
  norp02aa1n03x5               g218(.a(new_n311), .b(new_n313), .o1(\s[31] ));
  inv000aa1d42x5               g219(.a(new_n106), .o1(new_n315));
  xnbna2aa1n03x5               g220(.a(new_n103), .b(new_n107), .c(new_n315), .out0(\s[3] ));
  norb02aa1n02x5               g221(.a(new_n105), .b(new_n104), .out0(new_n317));
  nanb03aa1n02x5               g222(.a(new_n103), .b(new_n107), .c(new_n315), .out0(new_n318));
  xnbna2aa1n03x5               g223(.a(new_n317), .b(new_n318), .c(new_n315), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g225(.a(new_n111), .o1(new_n321));
  norb02aa1n02x5               g226(.a(new_n114), .b(new_n113), .out0(new_n322));
  nanp03aa1n02x5               g227(.a(new_n110), .b(new_n321), .c(new_n112), .o1(new_n323));
  nanb03aa1n03x5               g228(.a(new_n124), .b(new_n323), .c(new_n114), .out0(new_n324));
  aoai13aa1n02x5               g229(.a(new_n324), .b(new_n322), .c(new_n321), .d(new_n323), .o1(\s[6] ));
  xnbna2aa1n03x5               g230(.a(new_n116), .b(new_n324), .c(new_n114), .out0(\s[7] ));
  nanb03aa1n02x5               g231(.a(new_n116), .b(new_n324), .c(new_n114), .out0(new_n327));
  xobna2aa1n03x5               g232(.a(new_n117), .b(new_n327), .c(new_n121), .out0(\s[8] ));
  xnbna2aa1n03x5               g233(.a(new_n181), .b(new_n147), .c(new_n99), .out0(\s[9] ));
endmodule


