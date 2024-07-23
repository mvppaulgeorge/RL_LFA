// Benchmark "adder" written by ABC on Wed Jul 17 17:23:59 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n323, new_n325, new_n326, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n04x5               g001(.a(\b[3] ), .b(\a[4] ), .o1(new_n97));
  nand22aa1n03x5               g002(.a(\b[3] ), .b(\a[4] ), .o1(new_n98));
  nor022aa1n04x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  aoi012aa1n02x5               g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\a[2] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[1] ), .o1(new_n102));
  nand02aa1n03x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  oaoi03aa1n03x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n03x5               g010(.a(new_n105), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n106));
  oai012aa1n04x7               g011(.a(new_n100), .b(new_n106), .c(new_n104), .o1(new_n107));
  aoi022aa1d18x5               g012(.a(\b[6] ), .b(\a[7] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n108));
  oai122aa1n02x7               g013(.a(new_n108), .b(\a[6] ), .c(\b[5] ), .d(\a[5] ), .e(\b[4] ), .o1(new_n109));
  xorc02aa1n02x5               g014(.a(\a[8] ), .b(\b[7] ), .out0(new_n110));
  nand42aa1n08x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  oai122aa1n12x5               g016(.a(new_n111), .b(\a[8] ), .c(\b[7] ), .d(\a[7] ), .e(\b[6] ), .o1(new_n112));
  norb03aa1n03x5               g017(.a(new_n110), .b(new_n109), .c(new_n112), .out0(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  oai022aa1n02x5               g019(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  oaih22aa1n04x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  aoi022aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(new_n117), .b(new_n118), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n116), .b(new_n119), .c(new_n112), .o1(new_n120));
  tech160nm_fixorc02aa1n03p5x5 g025(.a(\a[9] ), .b(\b[8] ), .out0(new_n121));
  aoai13aa1n06x5               g026(.a(new_n121), .b(new_n120), .c(new_n107), .d(new_n113), .o1(new_n122));
  xorc02aa1n12x5               g027(.a(\a[10] ), .b(\b[9] ), .out0(new_n123));
  inv000aa1d42x5               g028(.a(new_n123), .o1(new_n124));
  oaoi13aa1n04x5               g029(.a(new_n124), .b(new_n122), .c(\a[9] ), .d(\b[8] ), .o1(new_n125));
  nor002aa1d32x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nona22aa1n02x4               g031(.a(new_n122), .b(new_n123), .c(new_n126), .out0(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n125), .out0(\s[10] ));
  inv000aa1d42x5               g033(.a(\a[10] ), .o1(new_n129));
  inv040aa1d32x5               g034(.a(\b[9] ), .o1(new_n130));
  oaoi03aa1n12x5               g035(.a(new_n129), .b(new_n130), .c(new_n126), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  nand42aa1n16x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nor002aa1n06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  oai022aa1n02x5               g040(.a(new_n125), .b(new_n132), .c(new_n135), .d(new_n134), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n133), .b(new_n135), .out0(new_n137));
  nano22aa1n03x7               g042(.a(new_n125), .b(new_n131), .c(new_n137), .out0(new_n138));
  nanb02aa1n02x5               g043(.a(new_n138), .b(new_n136), .out0(\s[11] ));
  nor022aa1n08x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  tech160nm_fioai012aa1n03p5x5 g047(.a(new_n142), .b(new_n138), .c(new_n134), .o1(new_n143));
  nor043aa1n03x5               g048(.a(new_n138), .b(new_n142), .c(new_n134), .o1(new_n144));
  nanb02aa1n03x5               g049(.a(new_n144), .b(new_n143), .out0(\s[12] ));
  oao003aa1n02x5               g050(.a(new_n101), .b(new_n102), .c(new_n103), .carry(new_n146));
  nano23aa1n02x5               g051(.a(new_n97), .b(new_n99), .c(new_n105), .d(new_n98), .out0(new_n147));
  aobi12aa1n03x7               g052(.a(new_n100), .b(new_n147), .c(new_n146), .out0(new_n148));
  nona23aa1n09x5               g053(.a(new_n110), .b(new_n108), .c(new_n112), .d(new_n117), .out0(new_n149));
  oabi12aa1n09x5               g054(.a(new_n120), .b(new_n148), .c(new_n149), .out0(new_n150));
  nano23aa1n03x7               g055(.a(new_n140), .b(new_n135), .c(new_n141), .d(new_n133), .out0(new_n151));
  nand23aa1n04x5               g056(.a(new_n151), .b(new_n121), .c(new_n123), .o1(new_n152));
  nanb02aa1n06x5               g057(.a(new_n152), .b(new_n150), .out0(new_n153));
  nona23aa1n09x5               g058(.a(new_n133), .b(new_n141), .c(new_n140), .d(new_n135), .out0(new_n154));
  tech160nm_fioai012aa1n03p5x5 g059(.a(new_n141), .b(new_n140), .c(new_n135), .o1(new_n155));
  oai012aa1d24x5               g060(.a(new_n155), .b(new_n154), .c(new_n131), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  xorc02aa1n12x5               g062(.a(\a[13] ), .b(\b[12] ), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n153), .c(new_n157), .out0(\s[13] ));
  nor042aa1d18x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  inv040aa1n08x5               g065(.a(new_n160), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(new_n153), .b(new_n157), .o1(new_n162));
  nanp02aa1n03x5               g067(.a(new_n162), .b(new_n158), .o1(new_n163));
  xorc02aa1n12x5               g068(.a(\a[14] ), .b(\b[13] ), .out0(new_n164));
  xnbna2aa1n03x5               g069(.a(new_n164), .b(new_n163), .c(new_n161), .out0(\s[14] ));
  nanp02aa1n02x5               g070(.a(new_n164), .b(new_n158), .o1(new_n166));
  oaoi03aa1n12x5               g071(.a(\a[14] ), .b(\b[13] ), .c(new_n161), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n166), .c(new_n153), .d(new_n157), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n12x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand42aa1n16x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  nor042aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand42aa1n08x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n171), .c(new_n169), .d(new_n173), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(new_n169), .b(new_n173), .o1(new_n178));
  nona22aa1n02x4               g083(.a(new_n178), .b(new_n176), .c(new_n171), .out0(new_n179));
  nanp02aa1n02x5               g084(.a(new_n179), .b(new_n177), .o1(\s[16] ));
  nano23aa1d15x5               g085(.a(new_n174), .b(new_n171), .c(new_n175), .d(new_n172), .out0(new_n181));
  nand23aa1n06x5               g086(.a(new_n181), .b(new_n158), .c(new_n164), .o1(new_n182));
  nor042aa1n06x5               g087(.a(new_n182), .b(new_n152), .o1(new_n183));
  aoai13aa1n06x5               g088(.a(new_n183), .b(new_n120), .c(new_n107), .d(new_n113), .o1(new_n184));
  norb02aa1n09x5               g089(.a(new_n181), .b(new_n166), .out0(new_n185));
  tech160nm_fiao0012aa1n02p5x5 g090(.a(new_n174), .b(new_n171), .c(new_n175), .o(new_n186));
  aoi012aa1n12x5               g091(.a(new_n186), .b(new_n181), .c(new_n167), .o1(new_n187));
  aobi12aa1d24x5               g092(.a(new_n187), .b(new_n185), .c(new_n156), .out0(new_n188));
  nanp02aa1n06x5               g093(.a(new_n184), .b(new_n188), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g095(.a(\a[18] ), .o1(new_n191));
  inv030aa1d32x5               g096(.a(\a[17] ), .o1(new_n192));
  inv000aa1d42x5               g097(.a(\b[16] ), .o1(new_n193));
  oaoi03aa1n02x5               g098(.a(new_n192), .b(new_n193), .c(new_n189), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[17] ), .c(new_n191), .out0(\s[18] ));
  xroi22aa1d06x4               g100(.a(new_n192), .b(\b[16] ), .c(new_n191), .d(\b[17] ), .out0(new_n196));
  norp02aa1n02x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  nand22aa1n02x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  aoi013aa1n09x5               g103(.a(new_n197), .b(new_n198), .c(new_n192), .d(new_n193), .o1(new_n199));
  aob012aa1n03x5               g104(.a(new_n199), .b(new_n189), .c(new_n196), .out0(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand02aa1n03x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  nor042aa1n04x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nand02aa1d04x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanb02aa1n02x5               g112(.a(new_n206), .b(new_n207), .out0(new_n208));
  aoai13aa1n03x5               g113(.a(new_n208), .b(new_n203), .c(new_n200), .d(new_n205), .o1(new_n209));
  nona22aa1n02x4               g114(.a(new_n198), .b(\b[16] ), .c(\a[17] ), .out0(new_n210));
  oaib12aa1n02x5               g115(.a(new_n210), .b(\b[17] ), .c(new_n191), .out0(new_n211));
  aoai13aa1n02x5               g116(.a(new_n205), .b(new_n211), .c(new_n189), .d(new_n196), .o1(new_n212));
  nona22aa1n02x4               g117(.a(new_n212), .b(new_n208), .c(new_n203), .out0(new_n213));
  nanp02aa1n03x5               g118(.a(new_n209), .b(new_n213), .o1(\s[20] ));
  nano23aa1n03x7               g119(.a(new_n206), .b(new_n203), .c(new_n207), .d(new_n204), .out0(new_n215));
  nanp02aa1n02x5               g120(.a(new_n196), .b(new_n215), .o1(new_n216));
  nona23aa1n09x5               g121(.a(new_n204), .b(new_n207), .c(new_n206), .d(new_n203), .out0(new_n217));
  tech160nm_fioai012aa1n03p5x5 g122(.a(new_n207), .b(new_n206), .c(new_n203), .o1(new_n218));
  oai012aa1n18x5               g123(.a(new_n218), .b(new_n217), .c(new_n199), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n216), .c(new_n184), .d(new_n188), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  xorc02aa1n02x5               g129(.a(\a[22] ), .b(\b[21] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n227));
  aoi112aa1n03x4               g132(.a(new_n223), .b(new_n226), .c(new_n221), .d(new_n224), .o1(new_n228));
  nanb02aa1n03x5               g133(.a(new_n228), .b(new_n227), .out0(\s[22] ));
  inv000aa1d42x5               g134(.a(\a[21] ), .o1(new_n230));
  inv040aa1d32x5               g135(.a(\a[22] ), .o1(new_n231));
  xroi22aa1d06x4               g136(.a(new_n230), .b(\b[20] ), .c(new_n231), .d(\b[21] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(\b[21] ), .o1(new_n233));
  oao003aa1n06x5               g138(.a(new_n231), .b(new_n233), .c(new_n223), .carry(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n219), .c(new_n232), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n232), .o1(new_n236));
  nona22aa1n09x5               g141(.a(new_n189), .b(new_n216), .c(new_n236), .out0(new_n237));
  tech160nm_fixnrc02aa1n05x5   g142(.a(\b[22] ), .b(\a[23] ), .out0(new_n238));
  xobna2aa1n03x5               g143(.a(new_n238), .b(new_n237), .c(new_n235), .out0(\s[23] ));
  and002aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o(new_n240));
  xorc02aa1n12x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  inv020aa1n02x5               g147(.a(new_n218), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n232), .b(new_n243), .c(new_n215), .d(new_n211), .o1(new_n244));
  nor042aa1n06x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  nona22aa1n02x4               g150(.a(new_n244), .b(new_n234), .c(new_n245), .out0(new_n246));
  inv000aa1n02x5               g151(.a(new_n246), .o1(new_n247));
  aoai13aa1n04x5               g152(.a(new_n242), .b(new_n240), .c(new_n237), .d(new_n247), .o1(new_n248));
  aoi112aa1n06x5               g153(.a(new_n242), .b(new_n240), .c(new_n237), .d(new_n247), .o1(new_n249));
  norb02aa1n03x4               g154(.a(new_n248), .b(new_n249), .out0(\s[24] ));
  oaib12aa1n09x5               g155(.a(new_n187), .b(new_n182), .c(new_n156), .out0(new_n251));
  norb02aa1n02x7               g156(.a(new_n241), .b(new_n238), .out0(new_n252));
  nano22aa1n03x5               g157(.a(new_n216), .b(new_n252), .c(new_n232), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n251), .c(new_n150), .d(new_n183), .o1(new_n254));
  aoai13aa1n06x5               g159(.a(new_n252), .b(new_n234), .c(new_n219), .d(new_n232), .o1(new_n255));
  inv000aa1n03x5               g160(.a(new_n245), .o1(new_n256));
  oaoi03aa1n03x5               g161(.a(\a[24] ), .b(\b[23] ), .c(new_n256), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  nanp03aa1n03x5               g163(.a(new_n254), .b(new_n255), .c(new_n258), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  tech160nm_fixorc02aa1n05x5   g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  nor042aa1n03x5               g167(.a(\b[25] ), .b(\a[26] ), .o1(new_n263));
  nand42aa1n03x5               g168(.a(\b[25] ), .b(\a[26] ), .o1(new_n264));
  norb02aa1n03x4               g169(.a(new_n264), .b(new_n263), .out0(new_n265));
  inv040aa1n03x5               g170(.a(new_n265), .o1(new_n266));
  aoai13aa1n03x5               g171(.a(new_n266), .b(new_n261), .c(new_n259), .d(new_n262), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n234), .o1(new_n268));
  inv000aa1n02x5               g173(.a(new_n252), .o1(new_n269));
  aoai13aa1n04x5               g174(.a(new_n258), .b(new_n269), .c(new_n244), .d(new_n268), .o1(new_n270));
  aoai13aa1n02x5               g175(.a(new_n262), .b(new_n270), .c(new_n189), .d(new_n253), .o1(new_n271));
  nona22aa1n02x4               g176(.a(new_n271), .b(new_n266), .c(new_n261), .out0(new_n272));
  nanp02aa1n03x5               g177(.a(new_n267), .b(new_n272), .o1(\s[26] ));
  norb02aa1n02x7               g178(.a(new_n262), .b(new_n266), .out0(new_n274));
  inv000aa1n02x5               g179(.a(new_n274), .o1(new_n275));
  nano23aa1n06x5               g180(.a(new_n275), .b(new_n216), .c(new_n252), .d(new_n232), .out0(new_n276));
  aoai13aa1n12x5               g181(.a(new_n276), .b(new_n251), .c(new_n150), .d(new_n183), .o1(new_n277));
  oai012aa1n02x5               g182(.a(new_n264), .b(new_n263), .c(new_n261), .o1(new_n278));
  aobi12aa1n06x5               g183(.a(new_n278), .b(new_n270), .c(new_n274), .out0(new_n279));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  norb02aa1n02x5               g186(.a(new_n281), .b(new_n280), .out0(new_n282));
  xnbna2aa1n03x5               g187(.a(new_n282), .b(new_n279), .c(new_n277), .out0(\s[27] ));
  norp02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .o1(new_n285));
  norb02aa1n02x5               g190(.a(new_n285), .b(new_n284), .out0(new_n286));
  oai112aa1n03x5               g191(.a(new_n279), .b(new_n277), .c(\b[26] ), .d(\a[27] ), .o1(new_n287));
  aoi012aa1n03x5               g192(.a(new_n286), .b(new_n287), .c(new_n281), .o1(new_n288));
  aobi12aa1n06x5               g193(.a(new_n276), .b(new_n184), .c(new_n188), .out0(new_n289));
  aoai13aa1n04x5               g194(.a(new_n278), .b(new_n275), .c(new_n255), .d(new_n258), .o1(new_n290));
  norp03aa1n02x5               g195(.a(new_n290), .b(new_n289), .c(new_n280), .o1(new_n291));
  nano22aa1n02x5               g196(.a(new_n291), .b(new_n281), .c(new_n286), .out0(new_n292));
  nor002aa1n02x5               g197(.a(new_n288), .b(new_n292), .o1(\s[28] ));
  nano23aa1n02x4               g198(.a(new_n280), .b(new_n284), .c(new_n285), .d(new_n281), .out0(new_n294));
  oai012aa1n03x5               g199(.a(new_n294), .b(new_n290), .c(new_n289), .o1(new_n295));
  aoi012aa1n02x5               g200(.a(new_n284), .b(new_n280), .c(new_n285), .o1(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x7               g203(.a(new_n294), .b(new_n279), .c(new_n277), .out0(new_n299));
  nano22aa1n03x5               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g207(.a(new_n297), .b(new_n282), .c(new_n286), .out0(new_n303));
  oai012aa1n03x5               g208(.a(new_n303), .b(new_n290), .c(new_n289), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n02x7               g212(.a(new_n303), .b(new_n279), .c(new_n277), .out0(new_n308));
  nano22aa1n03x5               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[30] ));
  nano23aa1n02x4               g215(.a(new_n306), .b(new_n297), .c(new_n286), .d(new_n282), .out0(new_n311));
  oai012aa1n03x5               g216(.a(new_n311), .b(new_n290), .c(new_n289), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[30] ), .b(\a[31] ), .out0(new_n314));
  tech160nm_fiaoi012aa1n02p5x5 g219(.a(new_n314), .b(new_n312), .c(new_n313), .o1(new_n315));
  aobi12aa1n02x7               g220(.a(new_n311), .b(new_n279), .c(new_n277), .out0(new_n316));
  nano22aa1n03x5               g221(.a(new_n316), .b(new_n313), .c(new_n314), .out0(new_n317));
  norp02aa1n03x5               g222(.a(new_n315), .b(new_n317), .o1(\s[31] ));
  xnrb03aa1n02x5               g223(.a(new_n104), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g224(.a(\a[3] ), .b(\b[2] ), .c(new_n104), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g227(.a(\a[5] ), .b(\b[4] ), .c(new_n148), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norp02aa1n02x5               g229(.a(\b[5] ), .b(\a[6] ), .o1(new_n325));
  oai012aa1n02x5               g230(.a(new_n111), .b(new_n323), .c(new_n325), .o1(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g232(.a(\a[7] ), .b(\b[6] ), .c(new_n326), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g234(.a(new_n150), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


