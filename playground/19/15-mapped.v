// Benchmark "adder" written by ABC on Wed Jul 17 21:48:23 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n322, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor042aa1n09x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv040aa1d32x5               g003(.a(\a[4] ), .o1(new_n99));
  inv000aa1d48x5               g004(.a(\b[3] ), .o1(new_n100));
  nand42aa1n16x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  nand42aa1n04x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand02aa1n04x5               g007(.a(new_n101), .b(new_n102), .o1(new_n103));
  xnrc02aa1n12x5               g008(.a(\b[2] ), .b(\a[3] ), .out0(new_n104));
  nand02aa1n06x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand02aa1d28x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  nor042aa1d18x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  oaih12aa1n12x5               g012(.a(new_n105), .b(new_n107), .c(new_n106), .o1(new_n108));
  aoi112aa1n03x5               g013(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n109));
  norb02aa1n06x4               g014(.a(new_n101), .b(new_n109), .out0(new_n110));
  oai013aa1d12x5               g015(.a(new_n110), .b(new_n104), .c(new_n108), .d(new_n103), .o1(new_n111));
  xnrc02aa1n06x5               g016(.a(\b[5] ), .b(\a[6] ), .out0(new_n112));
  nor002aa1d32x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nand02aa1d28x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nor002aa1d32x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nand42aa1n16x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nona23aa1n09x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  tech160nm_fixnrc02aa1n02p5x5 g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  nor043aa1n06x5               g023(.a(new_n117), .b(new_n118), .c(new_n112), .o1(new_n119));
  nano23aa1d15x5               g024(.a(new_n113), .b(new_n115), .c(new_n116), .d(new_n114), .out0(new_n120));
  inv040aa1d32x5               g025(.a(\a[5] ), .o1(new_n121));
  inv040aa1d32x5               g026(.a(\b[4] ), .o1(new_n122));
  nand02aa1n04x5               g027(.a(new_n122), .b(new_n121), .o1(new_n123));
  oaoi03aa1n12x5               g028(.a(\a[6] ), .b(\b[5] ), .c(new_n123), .o1(new_n124));
  ao0012aa1n12x5               g029(.a(new_n113), .b(new_n115), .c(new_n114), .o(new_n125));
  aoi012aa1d24x5               g030(.a(new_n125), .b(new_n120), .c(new_n124), .o1(new_n126));
  inv040aa1n08x5               g031(.a(new_n126), .o1(new_n127));
  xorc02aa1n12x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n127), .c(new_n111), .d(new_n119), .o1(new_n129));
  oaib12aa1n02x5               g034(.a(new_n97), .b(new_n98), .c(new_n129), .out0(new_n130));
  nona22aa1n03x5               g035(.a(new_n129), .b(new_n98), .c(new_n97), .out0(new_n131));
  nanp02aa1n02x5               g036(.a(new_n130), .b(new_n131), .o1(\s[10] ));
  inv000aa1d42x5               g037(.a(\a[10] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(\b[9] ), .o1(new_n134));
  nor002aa1d32x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand02aa1d28x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  oai112aa1n03x5               g042(.a(new_n131), .b(new_n137), .c(new_n134), .d(new_n133), .o1(new_n138));
  oaoi13aa1n02x5               g043(.a(new_n137), .b(new_n131), .c(new_n133), .d(new_n134), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(\s[11] ));
  inv040aa1n06x5               g045(.a(new_n135), .o1(new_n141));
  nor002aa1d32x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand02aa1d20x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n138), .c(new_n141), .out0(\s[12] ));
  nand02aa1d04x5               g050(.a(new_n111), .b(new_n119), .o1(new_n146));
  nona23aa1d18x5               g051(.a(new_n143), .b(new_n136), .c(new_n135), .d(new_n142), .out0(new_n147));
  oaoi03aa1n09x5               g052(.a(new_n133), .b(new_n134), .c(new_n98), .o1(new_n148));
  oaoi03aa1n09x5               g053(.a(\a[12] ), .b(\b[11] ), .c(new_n141), .o1(new_n149));
  oabi12aa1n18x5               g054(.a(new_n149), .b(new_n147), .c(new_n148), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  xorc02aa1n02x5               g056(.a(\a[10] ), .b(\b[9] ), .out0(new_n152));
  nano23aa1n06x5               g057(.a(new_n135), .b(new_n142), .c(new_n143), .d(new_n136), .out0(new_n153));
  nanp03aa1n03x5               g058(.a(new_n153), .b(new_n152), .c(new_n128), .o1(new_n154));
  aoai13aa1n06x5               g059(.a(new_n151), .b(new_n154), .c(new_n146), .d(new_n126), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g061(.a(\a[14] ), .o1(new_n157));
  nor042aa1n04x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  xnrc02aa1n12x5               g063(.a(\b[12] ), .b(\a[13] ), .out0(new_n159));
  aoib12aa1n06x5               g064(.a(new_n158), .b(new_n155), .c(new_n159), .out0(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(new_n157), .out0(\s[14] ));
  xorc02aa1n02x5               g066(.a(\a[15] ), .b(\b[14] ), .out0(new_n162));
  inv000aa1d42x5               g067(.a(\b[13] ), .o1(new_n163));
  tech160nm_fioaoi03aa1n05x5   g068(.a(new_n157), .b(new_n163), .c(new_n158), .o1(new_n164));
  inv000aa1n02x5               g069(.a(new_n164), .o1(new_n165));
  tech160nm_fixnrc02aa1n04x5   g070(.a(\b[13] ), .b(\a[14] ), .out0(new_n166));
  nor042aa1n06x5               g071(.a(new_n166), .b(new_n159), .o1(new_n167));
  aoai13aa1n06x5               g072(.a(new_n162), .b(new_n165), .c(new_n155), .d(new_n167), .o1(new_n168));
  aoi112aa1n02x5               g073(.a(new_n165), .b(new_n162), .c(new_n155), .d(new_n167), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(\s[15] ));
  orn002aa1n03x5               g075(.a(\a[15] ), .b(\b[14] ), .o(new_n171));
  xnrc02aa1n12x5               g076(.a(\b[15] ), .b(\a[16] ), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n168), .c(new_n171), .out0(\s[16] ));
  nanp02aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nano22aa1n12x5               g080(.a(new_n172), .b(new_n171), .c(new_n175), .out0(new_n176));
  nano22aa1n03x5               g081(.a(new_n154), .b(new_n167), .c(new_n176), .out0(new_n177));
  aoai13aa1n06x5               g082(.a(new_n177), .b(new_n127), .c(new_n111), .d(new_n119), .o1(new_n178));
  aoai13aa1n06x5               g083(.a(new_n176), .b(new_n165), .c(new_n150), .d(new_n167), .o1(new_n179));
  oao003aa1n06x5               g084(.a(\a[16] ), .b(\b[15] ), .c(new_n171), .carry(new_n180));
  nanp03aa1d12x5               g085(.a(new_n178), .b(new_n179), .c(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g087(.a(\a[18] ), .o1(new_n183));
  inv040aa1d32x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n03x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  norb03aa1n03x5               g092(.a(new_n128), .b(new_n147), .c(new_n97), .out0(new_n188));
  nand23aa1n03x5               g093(.a(new_n188), .b(new_n167), .c(new_n176), .o1(new_n189));
  aoi012aa1n12x5               g094(.a(new_n189), .b(new_n146), .c(new_n126), .o1(new_n190));
  inv030aa1n02x5               g095(.a(new_n176), .o1(new_n191));
  oao003aa1n03x5               g096(.a(new_n133), .b(new_n134), .c(new_n98), .carry(new_n192));
  aoai13aa1n04x5               g097(.a(new_n167), .b(new_n149), .c(new_n153), .d(new_n192), .o1(new_n193));
  aoai13aa1n06x5               g098(.a(new_n180), .b(new_n191), .c(new_n193), .d(new_n164), .o1(new_n194));
  xroi22aa1d06x4               g099(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n195));
  oai012aa1n06x5               g100(.a(new_n195), .b(new_n194), .c(new_n190), .o1(new_n196));
  oaih22aa1d12x5               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  oaib12aa1n18x5               g102(.a(new_n197), .b(new_n183), .c(\b[17] ), .out0(new_n198));
  nor002aa1d32x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanp02aa1n24x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n196), .c(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g109(.a(new_n199), .o1(new_n205));
  tech160nm_fiaoi012aa1n02p5x5 g110(.a(new_n201), .b(new_n196), .c(new_n198), .o1(new_n206));
  nor002aa1d32x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nand02aa1d16x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  nano22aa1n02x4               g114(.a(new_n206), .b(new_n205), .c(new_n209), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n185), .b(new_n184), .o1(new_n211));
  oaoi03aa1n12x5               g116(.a(\a[18] ), .b(\b[17] ), .c(new_n211), .o1(new_n212));
  aoai13aa1n03x5               g117(.a(new_n202), .b(new_n212), .c(new_n181), .d(new_n195), .o1(new_n213));
  aoi012aa1n03x5               g118(.a(new_n209), .b(new_n213), .c(new_n205), .o1(new_n214));
  nor002aa1n02x5               g119(.a(new_n214), .b(new_n210), .o1(\s[20] ));
  nano23aa1n09x5               g120(.a(new_n199), .b(new_n207), .c(new_n208), .d(new_n200), .out0(new_n216));
  nand22aa1n09x5               g121(.a(new_n195), .b(new_n216), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  oai012aa1n06x5               g123(.a(new_n218), .b(new_n194), .c(new_n190), .o1(new_n219));
  nona23aa1d18x5               g124(.a(new_n208), .b(new_n200), .c(new_n199), .d(new_n207), .out0(new_n220));
  aoi012aa1n12x5               g125(.a(new_n207), .b(new_n199), .c(new_n208), .o1(new_n221));
  oai012aa1d24x5               g126(.a(new_n221), .b(new_n220), .c(new_n198), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  nor042aa1n06x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  nanp02aa1n02x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  xnbna2aa1n03x5               g131(.a(new_n226), .b(new_n219), .c(new_n223), .out0(\s[21] ));
  inv000aa1n09x5               g132(.a(new_n224), .o1(new_n228));
  aobi12aa1n06x5               g133(.a(new_n226), .b(new_n219), .c(new_n223), .out0(new_n229));
  xnrc02aa1n12x5               g134(.a(\b[21] ), .b(\a[22] ), .out0(new_n230));
  nano22aa1n02x4               g135(.a(new_n229), .b(new_n228), .c(new_n230), .out0(new_n231));
  aoai13aa1n06x5               g136(.a(new_n226), .b(new_n222), .c(new_n181), .d(new_n218), .o1(new_n232));
  aoi012aa1n03x5               g137(.a(new_n230), .b(new_n232), .c(new_n228), .o1(new_n233));
  nor002aa1n02x5               g138(.a(new_n233), .b(new_n231), .o1(\s[22] ));
  nano22aa1n03x7               g139(.a(new_n230), .b(new_n228), .c(new_n225), .out0(new_n235));
  and003aa1n02x5               g140(.a(new_n195), .b(new_n235), .c(new_n216), .o(new_n236));
  oai012aa1n06x5               g141(.a(new_n236), .b(new_n194), .c(new_n190), .o1(new_n237));
  oao003aa1n02x5               g142(.a(\a[22] ), .b(\b[21] ), .c(new_n228), .carry(new_n238));
  inv000aa1n02x5               g143(.a(new_n238), .o1(new_n239));
  aoi012aa1n02x5               g144(.a(new_n239), .b(new_n222), .c(new_n235), .o1(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[22] ), .b(\a[23] ), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  xnbna2aa1n03x5               g147(.a(new_n242), .b(new_n237), .c(new_n240), .out0(\s[23] ));
  nor042aa1n06x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  tech160nm_fiaoi012aa1n02p5x5 g150(.a(new_n241), .b(new_n237), .c(new_n240), .o1(new_n246));
  tech160nm_fixnrc02aa1n04x5   g151(.a(\b[23] ), .b(\a[24] ), .out0(new_n247));
  nano22aa1n03x7               g152(.a(new_n246), .b(new_n245), .c(new_n247), .out0(new_n248));
  inv030aa1n02x5               g153(.a(new_n240), .o1(new_n249));
  aoai13aa1n03x5               g154(.a(new_n242), .b(new_n249), .c(new_n181), .d(new_n236), .o1(new_n250));
  aoi012aa1n03x5               g155(.a(new_n247), .b(new_n250), .c(new_n245), .o1(new_n251));
  norp02aa1n03x5               g156(.a(new_n251), .b(new_n248), .o1(\s[24] ));
  nor042aa1n03x5               g157(.a(new_n247), .b(new_n241), .o1(new_n253));
  nano22aa1n03x7               g158(.a(new_n217), .b(new_n235), .c(new_n253), .out0(new_n254));
  tech160nm_fioai012aa1n05x5   g159(.a(new_n254), .b(new_n194), .c(new_n190), .o1(new_n255));
  inv020aa1n04x5               g160(.a(new_n221), .o1(new_n256));
  aoai13aa1n06x5               g161(.a(new_n235), .b(new_n256), .c(new_n216), .d(new_n212), .o1(new_n257));
  inv040aa1n03x5               g162(.a(new_n253), .o1(new_n258));
  oao003aa1n06x5               g163(.a(\a[24] ), .b(\b[23] ), .c(new_n245), .carry(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n258), .c(new_n257), .d(new_n238), .o1(new_n260));
  xnrc02aa1n12x5               g165(.a(\b[24] ), .b(\a[25] ), .out0(new_n261));
  aoib12aa1n06x5               g166(.a(new_n261), .b(new_n255), .c(new_n260), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n261), .o1(new_n263));
  aoi112aa1n03x4               g168(.a(new_n263), .b(new_n260), .c(new_n181), .d(new_n254), .o1(new_n264));
  norp02aa1n02x5               g169(.a(new_n262), .b(new_n264), .o1(\s[25] ));
  nor042aa1n03x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  xnrc02aa1n12x5               g172(.a(\b[25] ), .b(\a[26] ), .out0(new_n268));
  nano22aa1n02x4               g173(.a(new_n262), .b(new_n267), .c(new_n268), .out0(new_n269));
  aoai13aa1n03x5               g174(.a(new_n263), .b(new_n260), .c(new_n181), .d(new_n254), .o1(new_n270));
  aoi012aa1n03x5               g175(.a(new_n268), .b(new_n270), .c(new_n267), .o1(new_n271));
  norp02aa1n03x5               g176(.a(new_n271), .b(new_n269), .o1(\s[26] ));
  nor042aa1n09x5               g177(.a(new_n268), .b(new_n261), .o1(new_n273));
  nano32aa1n03x7               g178(.a(new_n217), .b(new_n273), .c(new_n235), .d(new_n253), .out0(new_n274));
  oai012aa1n09x5               g179(.a(new_n274), .b(new_n194), .c(new_n190), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .c(new_n267), .carry(new_n276));
  aobi12aa1n06x5               g181(.a(new_n276), .b(new_n260), .c(new_n273), .out0(new_n277));
  xorc02aa1n12x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  xnbna2aa1n03x5               g183(.a(new_n278), .b(new_n275), .c(new_n277), .out0(\s[27] ));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  inv040aa1n03x5               g185(.a(new_n280), .o1(new_n281));
  aobi12aa1n02x7               g186(.a(new_n278), .b(new_n275), .c(new_n277), .out0(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  nano22aa1n03x5               g188(.a(new_n282), .b(new_n281), .c(new_n283), .out0(new_n284));
  aoai13aa1n03x5               g189(.a(new_n253), .b(new_n239), .c(new_n222), .d(new_n235), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n273), .o1(new_n286));
  aoai13aa1n06x5               g191(.a(new_n276), .b(new_n286), .c(new_n285), .d(new_n259), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n278), .b(new_n287), .c(new_n181), .d(new_n274), .o1(new_n288));
  aoi012aa1n03x5               g193(.a(new_n283), .b(new_n288), .c(new_n281), .o1(new_n289));
  norp02aa1n03x5               g194(.a(new_n289), .b(new_n284), .o1(\s[28] ));
  norb02aa1n02x5               g195(.a(new_n278), .b(new_n283), .out0(new_n291));
  aobi12aa1n02x7               g196(.a(new_n291), .b(new_n275), .c(new_n277), .out0(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n281), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  nano22aa1n03x5               g199(.a(new_n292), .b(new_n293), .c(new_n294), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n291), .b(new_n287), .c(new_n181), .d(new_n274), .o1(new_n296));
  aoi012aa1n03x5               g201(.a(new_n294), .b(new_n296), .c(new_n293), .o1(new_n297));
  norp02aa1n03x5               g202(.a(new_n297), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g204(.a(new_n278), .b(new_n294), .c(new_n283), .out0(new_n300));
  aobi12aa1n02x7               g205(.a(new_n300), .b(new_n275), .c(new_n277), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  nano22aa1n02x4               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  aoai13aa1n06x5               g209(.a(new_n300), .b(new_n287), .c(new_n181), .d(new_n274), .o1(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n03x5               g211(.a(new_n306), .b(new_n304), .o1(\s[30] ));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  norb02aa1n02x5               g213(.a(new_n300), .b(new_n303), .out0(new_n309));
  aobi12aa1n02x7               g214(.a(new_n309), .b(new_n275), .c(new_n277), .out0(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n311));
  nano22aa1n02x4               g216(.a(new_n310), .b(new_n308), .c(new_n311), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n309), .b(new_n287), .c(new_n181), .d(new_n274), .o1(new_n313));
  aoi012aa1n03x5               g218(.a(new_n308), .b(new_n313), .c(new_n311), .o1(new_n314));
  nor002aa1n02x5               g219(.a(new_n314), .b(new_n312), .o1(\s[31] ));
  xnrb03aa1n02x5               g220(.a(new_n108), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g221(.a(\a[3] ), .b(\b[2] ), .c(new_n108), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g223(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g224(.a(new_n121), .b(new_n122), .c(new_n111), .o1(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g226(.a(\a[6] ), .b(\b[5] ), .c(new_n320), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g228(.a(new_n115), .b(new_n322), .c(new_n116), .o1(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g230(.a(new_n128), .b(new_n146), .c(new_n126), .out0(\s[9] ));
endmodule


