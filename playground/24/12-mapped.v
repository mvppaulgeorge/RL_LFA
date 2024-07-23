// Benchmark "adder" written by ABC on Thu Jul 18 00:20:29 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n148, new_n149,
    new_n150, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n319,
    new_n320, new_n321, new_n323, new_n325, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[4] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[3] ), .o1(new_n98));
  nand02aa1n03x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(new_n99), .b(new_n100), .o1(new_n101));
  tech160nm_fixnrc02aa1n04x5   g006(.a(\b[2] ), .b(\a[3] ), .out0(new_n102));
  nand42aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand22aa1n04x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  nor042aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  oai012aa1n09x5               g010(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n106));
  aoi112aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n107));
  norb02aa1n02x5               g012(.a(new_n99), .b(new_n107), .out0(new_n108));
  oai013aa1n09x5               g013(.a(new_n108), .b(new_n102), .c(new_n106), .d(new_n101), .o1(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .out0(new_n110));
  norp02aa1n04x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand22aa1n06x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor042aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  tech160nm_finand02aa1n03p5x5 g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1d18x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n115), .b(new_n116), .c(new_n110), .o1(new_n117));
  oaih22aa1n04x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  aob012aa1n09x5               g023(.a(new_n118), .b(\b[5] ), .c(\a[6] ), .out0(new_n119));
  tech160nm_fiao0012aa1n02p5x5 g024(.a(new_n111), .b(new_n113), .c(new_n112), .o(new_n120));
  oabi12aa1n18x5               g025(.a(new_n120), .b(new_n115), .c(new_n119), .out0(new_n121));
  aoi012aa1n12x5               g026(.a(new_n121), .b(new_n109), .c(new_n117), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand02aa1d04x5               g029(.a(new_n109), .b(new_n117), .o1(new_n125));
  inv040aa1n02x5               g030(.a(new_n121), .o1(new_n126));
  nor042aa1d18x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nanp02aa1n12x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nor002aa1d32x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nand42aa1n03x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nano23aa1d15x5               g035(.a(new_n127), .b(new_n129), .c(new_n130), .d(new_n128), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  oai012aa1d24x5               g037(.a(new_n128), .b(new_n129), .c(new_n127), .o1(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n132), .c(new_n125), .d(new_n126), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n16x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand42aa1n06x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  aoi012aa1n02x5               g042(.a(new_n136), .b(new_n134), .c(new_n137), .o1(new_n138));
  xnrb03aa1n02x5               g043(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n24x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n10x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nona23aa1d18x5               g046(.a(new_n141), .b(new_n137), .c(new_n136), .d(new_n140), .out0(new_n142));
  aoi012aa1d18x5               g047(.a(new_n140), .b(new_n136), .c(new_n141), .o1(new_n143));
  oai012aa1d24x5               g048(.a(new_n143), .b(new_n142), .c(new_n133), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  oai013aa1n03x5               g050(.a(new_n145), .b(new_n122), .c(new_n132), .d(new_n142), .o1(new_n146));
  xorb03aa1n02x5               g051(.a(new_n146), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g052(.a(\a[13] ), .o1(new_n148));
  inv000aa1d42x5               g053(.a(\b[12] ), .o1(new_n149));
  oaoi03aa1n03x5               g054(.a(new_n148), .b(new_n149), .c(new_n146), .o1(new_n150));
  xnrb03aa1n03x5               g055(.a(new_n150), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n06x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nor022aa1n06x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nand42aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nano23aa1n06x5               g060(.a(new_n152), .b(new_n154), .c(new_n155), .d(new_n153), .out0(new_n156));
  oa0012aa1n06x5               g061(.a(new_n155), .b(new_n154), .c(new_n152), .o(new_n157));
  aoi012aa1n02x5               g062(.a(new_n157), .b(new_n144), .c(new_n156), .o1(new_n158));
  inv040aa1n03x5               g063(.a(new_n122), .o1(new_n159));
  nona23aa1n02x4               g064(.a(new_n155), .b(new_n153), .c(new_n152), .d(new_n154), .out0(new_n160));
  nona32aa1n02x4               g065(.a(new_n159), .b(new_n160), .c(new_n142), .d(new_n132), .out0(new_n161));
  nor042aa1n09x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  tech160nm_finand02aa1n03p5x5 g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  xnbna2aa1n03x5               g069(.a(new_n164), .b(new_n161), .c(new_n158), .out0(\s[15] ));
  inv000aa1d42x5               g070(.a(new_n162), .o1(new_n166));
  inv000aa1n02x5               g071(.a(new_n164), .o1(new_n167));
  aoai13aa1n03x5               g072(.a(new_n166), .b(new_n167), .c(new_n161), .d(new_n158), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  nor042aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1n03x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  aoi012aa1n02x5               g076(.a(new_n170), .b(new_n162), .c(new_n171), .o1(new_n172));
  nano23aa1d15x5               g077(.a(new_n162), .b(new_n170), .c(new_n171), .d(new_n163), .out0(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n157), .c(new_n144), .d(new_n156), .o1(new_n174));
  nano23aa1n09x5               g079(.a(new_n136), .b(new_n140), .c(new_n141), .d(new_n137), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n156), .b(new_n175), .o1(new_n176));
  nano22aa1n03x5               g081(.a(new_n176), .b(new_n131), .c(new_n173), .out0(new_n177));
  aoai13aa1n06x5               g082(.a(new_n177), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n178));
  nand23aa1n06x5               g083(.a(new_n178), .b(new_n174), .c(new_n172), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g085(.a(\a[18] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\a[17] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[16] ), .o1(new_n183));
  oaoi03aa1n03x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  inv040aa1n03x5               g090(.a(new_n133), .o1(new_n186));
  inv000aa1n02x5               g091(.a(new_n143), .o1(new_n187));
  aoai13aa1n06x5               g092(.a(new_n156), .b(new_n187), .c(new_n175), .d(new_n186), .o1(new_n188));
  inv000aa1n02x5               g093(.a(new_n157), .o1(new_n189));
  inv000aa1d42x5               g094(.a(new_n173), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n172), .b(new_n190), .c(new_n188), .d(new_n189), .o1(new_n191));
  nona23aa1n09x5               g096(.a(new_n131), .b(new_n173), .c(new_n160), .d(new_n142), .out0(new_n192));
  aoi012aa1n12x5               g097(.a(new_n192), .b(new_n125), .c(new_n126), .o1(new_n193));
  xroi22aa1d04x5               g098(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n194));
  tech160nm_fioai012aa1n05x5   g099(.a(new_n194), .b(new_n191), .c(new_n193), .o1(new_n195));
  oai022aa1d24x5               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n18x5               g101(.a(new_n196), .b(new_n181), .c(\b[17] ), .out0(new_n197));
  xnrc02aa1n12x5               g102(.a(\b[18] ), .b(\a[19] ), .out0(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n199), .b(new_n195), .c(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n09x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  aoi012aa1n06x5               g108(.a(new_n198), .b(new_n195), .c(new_n197), .o1(new_n204));
  tech160nm_fixnrc02aa1n05x5   g109(.a(\b[19] ), .b(\a[20] ), .out0(new_n205));
  nano22aa1n03x7               g110(.a(new_n204), .b(new_n203), .c(new_n205), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n197), .o1(new_n207));
  aoai13aa1n03x5               g112(.a(new_n199), .b(new_n207), .c(new_n179), .d(new_n194), .o1(new_n208));
  tech160nm_fiaoi012aa1n02p5x5 g113(.a(new_n205), .b(new_n208), .c(new_n203), .o1(new_n209));
  norp02aa1n03x5               g114(.a(new_n209), .b(new_n206), .o1(\s[20] ));
  nor042aa1n03x5               g115(.a(new_n205), .b(new_n198), .o1(new_n211));
  and002aa1n06x5               g116(.a(new_n194), .b(new_n211), .o(new_n212));
  tech160nm_fioai012aa1n05x5   g117(.a(new_n212), .b(new_n191), .c(new_n193), .o1(new_n213));
  oao003aa1n02x5               g118(.a(\a[20] ), .b(\b[19] ), .c(new_n203), .carry(new_n214));
  oai013aa1d12x5               g119(.a(new_n214), .b(new_n198), .c(new_n205), .d(new_n197), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  xorc02aa1n12x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n213), .c(new_n216), .out0(\s[21] ));
  orn002aa1n24x5               g123(.a(\a[21] ), .b(\b[20] ), .o(new_n219));
  inv000aa1d42x5               g124(.a(new_n217), .o1(new_n220));
  aoi012aa1n03x5               g125(.a(new_n220), .b(new_n213), .c(new_n216), .o1(new_n221));
  xnrc02aa1n02x5               g126(.a(\b[21] ), .b(\a[22] ), .out0(new_n222));
  nano22aa1n02x4               g127(.a(new_n221), .b(new_n219), .c(new_n222), .out0(new_n223));
  aoai13aa1n03x5               g128(.a(new_n217), .b(new_n215), .c(new_n179), .d(new_n212), .o1(new_n224));
  tech160nm_fiaoi012aa1n02p5x5 g129(.a(new_n222), .b(new_n224), .c(new_n219), .o1(new_n225));
  norp02aa1n03x5               g130(.a(new_n225), .b(new_n223), .o1(\s[22] ));
  norb02aa1n02x5               g131(.a(new_n217), .b(new_n222), .out0(new_n227));
  oao003aa1n09x5               g132(.a(\a[22] ), .b(\b[21] ), .c(new_n219), .carry(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  tech160nm_fiaoi012aa1n04x5   g134(.a(new_n229), .b(new_n215), .c(new_n227), .o1(new_n230));
  and003aa1n02x5               g135(.a(new_n227), .b(new_n211), .c(new_n194), .o(new_n231));
  tech160nm_fioai012aa1n05x5   g136(.a(new_n231), .b(new_n191), .c(new_n193), .o1(new_n232));
  xorc02aa1n12x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xnbna2aa1n03x5               g138(.a(new_n233), .b(new_n232), .c(new_n230), .out0(\s[23] ));
  nor042aa1n03x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n233), .o1(new_n237));
  aoi012aa1n03x5               g142(.a(new_n237), .b(new_n232), .c(new_n230), .o1(new_n238));
  tech160nm_fixorc02aa1n03p5x5 g143(.a(\a[24] ), .b(\b[23] ), .out0(new_n239));
  inv040aa1n02x5               g144(.a(new_n239), .o1(new_n240));
  nano22aa1n02x4               g145(.a(new_n238), .b(new_n236), .c(new_n240), .out0(new_n241));
  inv040aa1n02x5               g146(.a(new_n230), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n233), .b(new_n242), .c(new_n179), .d(new_n231), .o1(new_n243));
  tech160nm_fiaoi012aa1n02p5x5 g148(.a(new_n240), .b(new_n243), .c(new_n236), .o1(new_n244));
  norp02aa1n03x5               g149(.a(new_n244), .b(new_n241), .o1(\s[24] ));
  inv020aa1n02x5               g150(.a(new_n212), .o1(new_n246));
  inv000aa1d42x5               g151(.a(\a[23] ), .o1(new_n247));
  inv000aa1d42x5               g152(.a(\a[24] ), .o1(new_n248));
  xroi22aa1d04x5               g153(.a(new_n247), .b(\b[22] ), .c(new_n248), .d(\b[23] ), .out0(new_n249));
  nano22aa1n06x5               g154(.a(new_n246), .b(new_n227), .c(new_n249), .out0(new_n250));
  tech160nm_fioai012aa1n05x5   g155(.a(new_n250), .b(new_n191), .c(new_n193), .o1(new_n251));
  oaoi03aa1n02x5               g156(.a(\a[24] ), .b(\b[23] ), .c(new_n236), .o1(new_n252));
  aoib12aa1n12x5               g157(.a(new_n252), .b(new_n249), .c(new_n228), .out0(new_n253));
  nano23aa1n06x5               g158(.a(new_n240), .b(new_n222), .c(new_n217), .d(new_n233), .out0(new_n254));
  nand02aa1d08x5               g159(.a(new_n254), .b(new_n215), .o1(new_n255));
  nand02aa1d16x5               g160(.a(new_n255), .b(new_n253), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  xnrc02aa1n12x5               g162(.a(\b[24] ), .b(\a[25] ), .out0(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n251), .c(new_n257), .out0(\s[25] ));
  nor042aa1n03x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  aoi012aa1n06x5               g167(.a(new_n258), .b(new_n251), .c(new_n257), .o1(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[25] ), .b(\a[26] ), .out0(new_n264));
  nano22aa1n03x7               g169(.a(new_n263), .b(new_n262), .c(new_n264), .out0(new_n265));
  aoai13aa1n03x5               g170(.a(new_n259), .b(new_n256), .c(new_n179), .d(new_n250), .o1(new_n266));
  tech160nm_fiaoi012aa1n02p5x5 g171(.a(new_n264), .b(new_n266), .c(new_n262), .o1(new_n267));
  norp02aa1n03x5               g172(.a(new_n267), .b(new_n265), .o1(\s[26] ));
  nor022aa1n04x5               g173(.a(new_n264), .b(new_n258), .o1(new_n269));
  inv000aa1n02x5               g174(.a(new_n269), .o1(new_n270));
  nano32aa1n03x7               g175(.a(new_n270), .b(new_n254), .c(new_n211), .d(new_n194), .out0(new_n271));
  tech160nm_fioai012aa1n03p5x5 g176(.a(new_n271), .b(new_n191), .c(new_n193), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n262), .carry(new_n273));
  aobi12aa1n12x5               g178(.a(new_n273), .b(new_n256), .c(new_n269), .out0(new_n274));
  nor002aa1n16x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  nanp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  nanb02aa1n06x5               g181(.a(new_n275), .b(new_n276), .out0(new_n277));
  xobna2aa1n03x5               g182(.a(new_n277), .b(new_n272), .c(new_n274), .out0(\s[27] ));
  inv000aa1d42x5               g183(.a(new_n275), .o1(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n273), .b(new_n270), .c(new_n255), .d(new_n253), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n276), .b(new_n281), .c(new_n179), .d(new_n271), .o1(new_n282));
  tech160nm_fiaoi012aa1n02p5x5 g187(.a(new_n280), .b(new_n282), .c(new_n279), .o1(new_n283));
  aoi022aa1n02x7               g188(.a(new_n272), .b(new_n274), .c(\b[26] ), .d(\a[27] ), .o1(new_n284));
  nano22aa1n03x5               g189(.a(new_n284), .b(new_n279), .c(new_n280), .out0(new_n285));
  norp02aa1n03x5               g190(.a(new_n283), .b(new_n285), .o1(\s[28] ));
  nor042aa1n03x5               g191(.a(new_n280), .b(new_n277), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n281), .c(new_n179), .d(new_n271), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n279), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n287), .o1(new_n292));
  tech160nm_fiaoi012aa1n02p5x5 g197(.a(new_n292), .b(new_n272), .c(new_n274), .o1(new_n293));
  nano22aa1n03x5               g198(.a(new_n293), .b(new_n289), .c(new_n290), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n291), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nor043aa1n03x5               g201(.a(new_n290), .b(new_n280), .c(new_n277), .o1(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n281), .c(new_n179), .d(new_n271), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n297), .o1(new_n302));
  tech160nm_fiaoi012aa1n02p5x5 g207(.a(new_n302), .b(new_n272), .c(new_n274), .o1(new_n303));
  nano22aa1n03x5               g208(.a(new_n303), .b(new_n299), .c(new_n300), .out0(new_n304));
  norp02aa1n03x5               g209(.a(new_n301), .b(new_n304), .o1(\s[30] ));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  norb03aa1n03x5               g211(.a(new_n287), .b(new_n300), .c(new_n290), .out0(new_n307));
  inv000aa1n02x5               g212(.a(new_n307), .o1(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n308), .b(new_n272), .c(new_n274), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n310));
  nano22aa1n03x5               g215(.a(new_n309), .b(new_n306), .c(new_n310), .out0(new_n311));
  aoai13aa1n03x5               g216(.a(new_n307), .b(new_n281), .c(new_n179), .d(new_n271), .o1(new_n312));
  tech160nm_fiaoi012aa1n02p5x5 g217(.a(new_n306), .b(new_n312), .c(new_n310), .o1(new_n313));
  norp02aa1n03x5               g218(.a(new_n313), .b(new_n311), .o1(\s[31] ));
  xnrb03aa1n02x5               g219(.a(new_n106), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n106), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g223(.a(\a[5] ), .o1(new_n319));
  inv000aa1d42x5               g224(.a(\b[4] ), .o1(new_n320));
  oaoi03aa1n02x5               g225(.a(new_n319), .b(new_n320), .c(new_n109), .o1(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g227(.a(\a[6] ), .b(\b[5] ), .c(new_n321), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g229(.a(new_n113), .b(new_n323), .c(new_n114), .o1(new_n325));
  xnrb03aa1n02x5               g230(.a(new_n325), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  inv000aa1d42x5               g231(.a(new_n129), .o1(new_n327));
  xnbna2aa1n03x5               g232(.a(new_n122), .b(new_n130), .c(new_n327), .out0(\s[9] ));
endmodule


