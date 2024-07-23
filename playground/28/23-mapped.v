// Benchmark "adder" written by ABC on Thu Jul 18 02:30:11 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n321, new_n324, new_n325, new_n327,
    new_n329;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nor042aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  oai012aa1n06x5               g007(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n103));
  tech160nm_fixnrc02aa1n02p5x5 g008(.a(\b[2] ), .b(\a[3] ), .out0(new_n104));
  inv040aa1d32x5               g009(.a(\a[4] ), .o1(new_n105));
  inv000aa1d42x5               g010(.a(\b[3] ), .o1(new_n106));
  nand02aa1n03x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nand42aa1n02x5               g013(.a(new_n107), .b(new_n108), .o1(new_n109));
  aoi112aa1n02x5               g014(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n110));
  norb02aa1n02x7               g015(.a(new_n107), .b(new_n110), .out0(new_n111));
  oai013aa1n06x5               g016(.a(new_n111), .b(new_n104), .c(new_n103), .d(new_n109), .o1(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  norp02aa1n04x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand22aa1n04x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nor022aa1n08x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand42aa1n03x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nona23aa1n09x5               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  xnrc02aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .out0(new_n119));
  nor043aa1n03x5               g024(.a(new_n118), .b(new_n119), .c(new_n113), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[6] ), .o1(new_n121));
  oai022aa1n02x7               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  oaib12aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(\b[5] ), .out0(new_n123));
  tech160nm_fiao0012aa1n02p5x5 g028(.a(new_n114), .b(new_n116), .c(new_n115), .o(new_n124));
  oabi12aa1n06x5               g029(.a(new_n124), .b(new_n118), .c(new_n123), .out0(new_n125));
  tech160nm_fixorc02aa1n04x5   g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n06x5               g031(.a(new_n126), .b(new_n125), .c(new_n112), .d(new_n120), .o1(new_n127));
  nor042aa1n09x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand02aa1n06x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n15x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g036(.a(new_n130), .o1(new_n132));
  aoai13aa1n12x5               g037(.a(new_n129), .b(new_n128), .c(new_n97), .d(new_n98), .o1(new_n133));
  nor042aa1n09x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand22aa1n04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  oaoi13aa1n03x5               g041(.a(new_n136), .b(new_n133), .c(new_n127), .d(new_n132), .o1(new_n137));
  oai112aa1n02x5               g042(.a(new_n133), .b(new_n136), .c(new_n127), .d(new_n132), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(\s[11] ));
  inv000aa1d42x5               g044(.a(\a[12] ), .o1(new_n140));
  norp02aa1n02x5               g045(.a(new_n137), .b(new_n134), .o1(new_n141));
  xorb03aa1n02x5               g046(.a(new_n141), .b(\b[11] ), .c(new_n140), .out0(\s[12] ));
  nor002aa1d32x5               g047(.a(\b[12] ), .b(\a[13] ), .o1(new_n143));
  nand42aa1n04x5               g048(.a(\b[12] ), .b(\a[13] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  nor042aa1n04x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand22aa1n09x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nona23aa1n09x5               g052(.a(new_n147), .b(new_n135), .c(new_n134), .d(new_n146), .out0(new_n148));
  tech160nm_fiaoi012aa1n03p5x5 g053(.a(new_n146), .b(new_n134), .c(new_n147), .o1(new_n149));
  tech160nm_fioai012aa1n03p5x5 g054(.a(new_n149), .b(new_n148), .c(new_n133), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n125), .b(new_n112), .c(new_n120), .o1(new_n151));
  nano23aa1n09x5               g056(.a(new_n134), .b(new_n146), .c(new_n147), .d(new_n135), .out0(new_n152));
  nano32aa1n03x7               g057(.a(new_n151), .b(new_n152), .c(new_n126), .d(new_n130), .out0(new_n153));
  tech160nm_fioai012aa1n05x5   g058(.a(new_n145), .b(new_n153), .c(new_n150), .o1(new_n154));
  norp03aa1n02x5               g059(.a(new_n153), .b(new_n150), .c(new_n145), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n154), .b(new_n155), .out0(\s[13] ));
  inv000aa1d42x5               g061(.a(new_n143), .o1(new_n157));
  nor042aa1n03x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n154), .c(new_n157), .out0(\s[14] ));
  nand22aa1n04x5               g066(.a(new_n112), .b(new_n120), .o1(new_n162));
  inv040aa1n02x5               g067(.a(new_n125), .o1(new_n163));
  nona23aa1n12x5               g068(.a(new_n159), .b(new_n144), .c(new_n143), .d(new_n158), .out0(new_n164));
  nano23aa1n06x5               g069(.a(new_n164), .b(new_n148), .c(new_n126), .d(new_n130), .out0(new_n165));
  aobi12aa1n02x5               g070(.a(new_n165), .b(new_n162), .c(new_n163), .out0(new_n166));
  inv000aa1n02x5               g071(.a(new_n133), .o1(new_n167));
  inv030aa1n02x5               g072(.a(new_n149), .o1(new_n168));
  inv040aa1n04x5               g073(.a(new_n164), .o1(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n168), .c(new_n152), .d(new_n167), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n158), .b(new_n143), .c(new_n159), .o1(new_n171));
  nano22aa1n03x7               g076(.a(new_n166), .b(new_n170), .c(new_n171), .out0(new_n172));
  xnrb03aa1n02x5               g077(.a(new_n172), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  oaoi03aa1n02x5               g078(.a(\a[15] ), .b(\b[14] ), .c(new_n172), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  nand23aa1n02x5               g080(.a(new_n126), .b(new_n145), .c(new_n160), .o1(new_n176));
  nor042aa1n04x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nand42aa1n03x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nor042aa1n04x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nand42aa1n06x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nano23aa1d15x5               g085(.a(new_n177), .b(new_n179), .c(new_n180), .d(new_n178), .out0(new_n181));
  nano32aa1n03x7               g086(.a(new_n176), .b(new_n181), .c(new_n152), .d(new_n130), .out0(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n125), .c(new_n112), .d(new_n120), .o1(new_n183));
  inv000aa1n02x5               g088(.a(new_n171), .o1(new_n184));
  aoai13aa1n06x5               g089(.a(new_n181), .b(new_n184), .c(new_n150), .d(new_n169), .o1(new_n185));
  aoi012aa1n12x5               g090(.a(new_n179), .b(new_n177), .c(new_n180), .o1(new_n186));
  nand23aa1n06x5               g091(.a(new_n183), .b(new_n185), .c(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g093(.a(\a[18] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaoi03aa1n03x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  nanp02aa1n02x5               g098(.a(new_n165), .b(new_n181), .o1(new_n194));
  aoi012aa1n12x5               g099(.a(new_n194), .b(new_n162), .c(new_n163), .o1(new_n195));
  inv000aa1d42x5               g100(.a(new_n181), .o1(new_n196));
  aoai13aa1n04x5               g101(.a(new_n186), .b(new_n196), .c(new_n170), .d(new_n171), .o1(new_n197));
  xroi22aa1d06x4               g102(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n198));
  oaih12aa1n02x5               g103(.a(new_n198), .b(new_n197), .c(new_n195), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\b[17] ), .o1(new_n200));
  oai022aa1d18x5               g105(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n201));
  oa0012aa1n02x5               g106(.a(new_n201), .b(new_n200), .c(new_n189), .o(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  xnrc02aa1n12x5               g108(.a(\b[18] ), .b(\a[19] ), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n199), .c(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g112(.a(\a[19] ), .o1(new_n208));
  inv000aa1d42x5               g113(.a(\b[18] ), .o1(new_n209));
  nand42aa1n02x5               g114(.a(new_n209), .b(new_n208), .o1(new_n210));
  aoi012aa1n03x5               g115(.a(new_n204), .b(new_n199), .c(new_n203), .o1(new_n211));
  xnrc02aa1n02x5               g116(.a(\b[19] ), .b(\a[20] ), .out0(new_n212));
  nano22aa1n03x5               g117(.a(new_n211), .b(new_n210), .c(new_n212), .out0(new_n213));
  aoai13aa1n03x5               g118(.a(new_n205), .b(new_n202), .c(new_n187), .d(new_n198), .o1(new_n214));
  tech160nm_fiaoi012aa1n02p5x5 g119(.a(new_n212), .b(new_n214), .c(new_n210), .o1(new_n215));
  norp02aa1n03x5               g120(.a(new_n215), .b(new_n213), .o1(\s[20] ));
  nona22aa1d18x5               g121(.a(new_n198), .b(new_n204), .c(new_n212), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  oaih12aa1n02x5               g123(.a(new_n218), .b(new_n197), .c(new_n195), .o1(new_n219));
  and002aa1n02x5               g124(.a(\b[19] ), .b(\a[20] ), .o(new_n220));
  oai122aa1n06x5               g125(.a(new_n201), .b(new_n208), .c(new_n209), .d(new_n189), .e(new_n200), .o1(new_n221));
  oai112aa1n06x5               g126(.a(new_n221), .b(new_n210), .c(\b[19] ), .d(\a[20] ), .o1(new_n222));
  norb02aa1n12x5               g127(.a(new_n222), .b(new_n220), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  xorc02aa1n12x5               g129(.a(\a[21] ), .b(\b[20] ), .out0(new_n225));
  xnbna2aa1n03x5               g130(.a(new_n225), .b(new_n219), .c(new_n224), .out0(\s[21] ));
  orn002aa1n12x5               g131(.a(\a[21] ), .b(\b[20] ), .o(new_n227));
  inv000aa1d42x5               g132(.a(new_n225), .o1(new_n228));
  aoi012aa1n03x5               g133(.a(new_n228), .b(new_n219), .c(new_n224), .o1(new_n229));
  xorc02aa1n12x5               g134(.a(\a[22] ), .b(\b[21] ), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nano22aa1n03x5               g136(.a(new_n229), .b(new_n227), .c(new_n231), .out0(new_n232));
  aoai13aa1n03x5               g137(.a(new_n225), .b(new_n223), .c(new_n187), .d(new_n218), .o1(new_n233));
  aoi012aa1n03x5               g138(.a(new_n231), .b(new_n233), .c(new_n227), .o1(new_n234));
  norp02aa1n03x5               g139(.a(new_n234), .b(new_n232), .o1(\s[22] ));
  nanp02aa1n06x5               g140(.a(new_n230), .b(new_n225), .o1(new_n236));
  nor042aa1n02x5               g141(.a(new_n217), .b(new_n236), .o1(new_n237));
  and002aa1n03x5               g142(.a(new_n230), .b(new_n225), .o(new_n238));
  oaoi03aa1n02x5               g143(.a(\a[22] ), .b(\b[21] ), .c(new_n227), .o1(new_n239));
  tech160nm_fiao0012aa1n02p5x5 g144(.a(new_n239), .b(new_n223), .c(new_n238), .o(new_n240));
  xnrc02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n240), .c(new_n187), .d(new_n237), .o1(new_n242));
  oai012aa1n02x5               g147(.a(new_n237), .b(new_n197), .c(new_n195), .o1(new_n243));
  nona22aa1n03x5               g148(.a(new_n243), .b(new_n240), .c(new_n241), .out0(new_n244));
  nanp02aa1n02x5               g149(.a(new_n242), .b(new_n244), .o1(\s[23] ));
  and002aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o(new_n246));
  xorc02aa1n02x5               g151(.a(\a[24] ), .b(\b[23] ), .out0(new_n247));
  nona22aa1n02x4               g152(.a(new_n244), .b(new_n247), .c(new_n246), .out0(new_n248));
  aoi112aa1n03x5               g153(.a(new_n241), .b(new_n240), .c(new_n187), .d(new_n237), .o1(new_n249));
  oaih12aa1n02x5               g154(.a(new_n247), .b(new_n249), .c(new_n246), .o1(new_n250));
  nanp02aa1n02x5               g155(.a(new_n250), .b(new_n248), .o1(\s[24] ));
  inv040aa1d30x5               g156(.a(\a[23] ), .o1(new_n252));
  inv040aa1d32x5               g157(.a(\a[24] ), .o1(new_n253));
  xroi22aa1d06x4               g158(.a(new_n252), .b(\b[22] ), .c(new_n253), .d(\b[23] ), .out0(new_n254));
  nano32aa1n06x5               g159(.a(new_n217), .b(new_n254), .c(new_n225), .d(new_n230), .out0(new_n255));
  oai012aa1n06x5               g160(.a(new_n255), .b(new_n197), .c(new_n195), .o1(new_n256));
  nona23aa1d18x5               g161(.a(new_n222), .b(new_n254), .c(new_n236), .d(new_n220), .out0(new_n257));
  nanb02aa1n02x5               g162(.a(\b[22] ), .b(new_n252), .out0(new_n258));
  oaoi03aa1n02x5               g163(.a(\a[24] ), .b(\b[23] ), .c(new_n258), .o1(new_n259));
  aoi012aa1n09x5               g164(.a(new_n259), .b(new_n254), .c(new_n239), .o1(new_n260));
  nand42aa1n16x5               g165(.a(new_n257), .b(new_n260), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  xnrc02aa1n12x5               g167(.a(\b[24] ), .b(\a[25] ), .out0(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n256), .c(new_n262), .out0(\s[25] ));
  nor042aa1n03x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  tech160nm_fiaoi012aa1n04x5   g172(.a(new_n263), .b(new_n256), .c(new_n262), .o1(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[25] ), .b(\a[26] ), .out0(new_n269));
  nano22aa1n03x5               g174(.a(new_n268), .b(new_n267), .c(new_n269), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n264), .b(new_n261), .c(new_n187), .d(new_n255), .o1(new_n271));
  aoi012aa1n03x5               g176(.a(new_n269), .b(new_n271), .c(new_n267), .o1(new_n272));
  norp02aa1n03x5               g177(.a(new_n272), .b(new_n270), .o1(\s[26] ));
  nor042aa1n02x5               g178(.a(new_n269), .b(new_n263), .o1(new_n274));
  nano32aa1n03x7               g179(.a(new_n217), .b(new_n274), .c(new_n238), .d(new_n254), .out0(new_n275));
  oai012aa1n12x5               g180(.a(new_n275), .b(new_n197), .c(new_n195), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .c(new_n267), .carry(new_n277));
  aobi12aa1n18x5               g182(.a(new_n277), .b(new_n261), .c(new_n274), .out0(new_n278));
  nor022aa1n12x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  nanp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  nanb02aa1n02x5               g185(.a(new_n279), .b(new_n280), .out0(new_n281));
  xobna2aa1n03x5               g186(.a(new_n281), .b(new_n276), .c(new_n278), .out0(\s[27] ));
  inv000aa1d42x5               g187(.a(new_n279), .o1(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  aoi022aa1n03x5               g189(.a(new_n276), .b(new_n278), .c(\b[26] ), .d(\a[27] ), .o1(new_n285));
  nano22aa1n03x5               g190(.a(new_n285), .b(new_n283), .c(new_n284), .out0(new_n286));
  inv000aa1n02x5               g191(.a(new_n274), .o1(new_n287));
  aoai13aa1n04x5               g192(.a(new_n277), .b(new_n287), .c(new_n257), .d(new_n260), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n280), .b(new_n288), .c(new_n187), .d(new_n275), .o1(new_n289));
  aoi012aa1n03x5               g194(.a(new_n284), .b(new_n289), .c(new_n283), .o1(new_n290));
  norp02aa1n03x5               g195(.a(new_n290), .b(new_n286), .o1(\s[28] ));
  nor042aa1n04x5               g196(.a(new_n284), .b(new_n281), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n288), .c(new_n187), .d(new_n275), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[28] ), .b(\a[29] ), .out0(new_n295));
  tech160nm_fiaoi012aa1n05x5   g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n292), .o1(new_n297));
  tech160nm_fiaoi012aa1n05x5   g202(.a(new_n297), .b(new_n276), .c(new_n278), .o1(new_n298));
  nano22aa1n03x5               g203(.a(new_n298), .b(new_n294), .c(new_n295), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n296), .b(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nor043aa1n03x5               g206(.a(new_n295), .b(new_n284), .c(new_n281), .o1(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n288), .c(new_n187), .d(new_n275), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .out0(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  inv000aa1d42x5               g211(.a(new_n302), .o1(new_n307));
  aoi012aa1n06x5               g212(.a(new_n307), .b(new_n276), .c(new_n278), .o1(new_n308));
  nano22aa1n03x5               g213(.a(new_n308), .b(new_n304), .c(new_n305), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n306), .b(new_n309), .o1(\s[30] ));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  norb03aa1n03x5               g216(.a(new_n292), .b(new_n305), .c(new_n295), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n288), .c(new_n187), .d(new_n275), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n314));
  tech160nm_fiaoi012aa1n02p5x5 g219(.a(new_n311), .b(new_n313), .c(new_n314), .o1(new_n315));
  inv000aa1n02x5               g220(.a(new_n312), .o1(new_n316));
  aoi012aa1n06x5               g221(.a(new_n316), .b(new_n276), .c(new_n278), .o1(new_n317));
  nano22aa1n03x5               g222(.a(new_n317), .b(new_n311), .c(new_n314), .out0(new_n318));
  norp02aa1n03x5               g223(.a(new_n315), .b(new_n318), .o1(\s[31] ));
  xnrb03aa1n02x5               g224(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g225(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g227(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g228(.a(\b[4] ), .b(\a[5] ), .o1(new_n324));
  oai012aa1n02x5               g229(.a(new_n324), .b(new_n112), .c(new_n119), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[5] ), .c(new_n121), .out0(\s[6] ));
  oaoi03aa1n02x5               g231(.a(\a[6] ), .b(\b[5] ), .c(new_n325), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g233(.a(new_n116), .b(new_n327), .c(new_n117), .o1(new_n329));
  xnrb03aa1n02x5               g234(.a(new_n329), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g235(.a(new_n151), .b(\b[8] ), .c(new_n97), .out0(\s[9] ));
endmodule


