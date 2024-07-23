// Benchmark "adder" written by ABC on Thu Jul 18 04:52:06 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n312, new_n314, new_n316, new_n318, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nand42aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  and002aa1n02x7               g004(.a(\b[1] ), .b(\a[2] ), .o(new_n100));
  nor042aa1n04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand22aa1n12x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  nor002aa1n02x5               g007(.a(new_n101), .b(new_n102), .o1(new_n103));
  nor002aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand42aa1n08x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n03x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nor042aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  tech160nm_finand02aa1n05x5   g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n09x5               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nona23aa1d18x5               g014(.a(new_n106), .b(new_n109), .c(new_n103), .d(new_n100), .out0(new_n110));
  tech160nm_fioai012aa1n03p5x5 g015(.a(new_n105), .b(new_n107), .c(new_n104), .o1(new_n111));
  nor042aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand42aa1n20x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor042aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n08x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n09x5               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  nanp02aa1n04x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nor002aa1d32x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nor042aa1d18x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nanp02aa1n04x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  nano23aa1n03x7               g025(.a(new_n119), .b(new_n118), .c(new_n120), .d(new_n117), .out0(new_n121));
  nand02aa1n02x5               g026(.a(new_n121), .b(new_n116), .o1(new_n122));
  and002aa1n02x7               g027(.a(\b[5] ), .b(\a[6] ), .o(new_n123));
  oab012aa1n03x5               g028(.a(new_n123), .b(new_n118), .c(new_n119), .out0(new_n124));
  oai022aa1n03x5               g029(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n125));
  aoi022aa1n09x5               g030(.a(new_n116), .b(new_n124), .c(new_n113), .d(new_n125), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n126), .b(new_n122), .c(new_n110), .d(new_n111), .o1(new_n127));
  tech160nm_fioai012aa1n05x5   g032(.a(new_n98), .b(new_n127), .c(new_n99), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  oaoi03aa1n03x5               g034(.a(\a[10] ), .b(\b[9] ), .c(new_n128), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n03x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n03x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  nor042aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  aoai13aa1n02x7               g042(.a(new_n137), .b(new_n132), .c(new_n130), .d(new_n134), .o1(new_n138));
  nand42aa1n03x5               g043(.a(\b[9] ), .b(\a[10] ), .o1(new_n139));
  oaib12aa1n02x5               g044(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(new_n140));
  nanp03aa1n02x5               g045(.a(new_n140), .b(new_n139), .c(new_n134), .o1(new_n141));
  nona22aa1n02x4               g046(.a(new_n141), .b(new_n137), .c(new_n132), .out0(new_n142));
  nanp02aa1n03x5               g047(.a(new_n138), .b(new_n142), .o1(\s[12] ));
  nano23aa1n06x5               g048(.a(new_n132), .b(new_n135), .c(new_n136), .d(new_n133), .out0(new_n144));
  norp02aa1n02x5               g049(.a(\b[9] ), .b(\a[10] ), .o1(new_n145));
  oa0012aa1n06x5               g050(.a(new_n139), .b(new_n99), .c(new_n145), .o(new_n146));
  oaih12aa1n02x5               g051(.a(new_n136), .b(new_n135), .c(new_n132), .o1(new_n147));
  aob012aa1n02x5               g052(.a(new_n147), .b(new_n144), .c(new_n146), .out0(new_n148));
  norb02aa1n02x5               g053(.a(new_n139), .b(new_n145), .out0(new_n149));
  norb02aa1n02x5               g054(.a(new_n98), .b(new_n99), .out0(new_n150));
  nano32aa1n06x5               g055(.a(new_n137), .b(new_n150), .c(new_n149), .d(new_n134), .out0(new_n151));
  nor042aa1n09x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand42aa1n03x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n148), .c(new_n127), .d(new_n151), .o1(new_n155));
  aoi112aa1n02x5               g060(.a(new_n148), .b(new_n154), .c(new_n127), .d(new_n151), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(new_n152), .o1(new_n158));
  nor042aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  tech160nm_finand02aa1n03p5x5 g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(new_n161));
  xobna2aa1n03x5               g066(.a(new_n161), .b(new_n155), .c(new_n158), .out0(\s[14] ));
  nor042aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanp02aa1n04x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  nano23aa1n09x5               g070(.a(new_n152), .b(new_n159), .c(new_n160), .d(new_n153), .out0(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n148), .c(new_n127), .d(new_n151), .o1(new_n167));
  aoi012aa1n02x5               g072(.a(new_n159), .b(new_n152), .c(new_n160), .o1(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n165), .b(new_n167), .c(new_n168), .out0(\s[15] ));
  tech160nm_finand02aa1n03p5x5 g074(.a(new_n167), .b(new_n168), .o1(new_n170));
  nor042aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanp02aa1n04x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nanb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(new_n173));
  aoai13aa1n03x5               g078(.a(new_n173), .b(new_n163), .c(new_n170), .d(new_n165), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(new_n170), .b(new_n165), .o1(new_n175));
  nona22aa1n02x4               g080(.a(new_n175), .b(new_n173), .c(new_n163), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n174), .o1(\s[16] ));
  nano23aa1n03x7               g082(.a(new_n163), .b(new_n171), .c(new_n172), .d(new_n164), .out0(new_n178));
  nand02aa1d04x5               g083(.a(new_n178), .b(new_n166), .o1(new_n179));
  nano32aa1n03x7               g084(.a(new_n179), .b(new_n144), .c(new_n150), .d(new_n149), .out0(new_n180));
  nanp02aa1n09x5               g085(.a(new_n127), .b(new_n180), .o1(new_n181));
  nand22aa1n03x5               g086(.a(new_n144), .b(new_n146), .o1(new_n182));
  oaoi03aa1n02x5               g087(.a(\a[14] ), .b(\b[13] ), .c(new_n158), .o1(new_n183));
  oai022aa1n02x5               g088(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n184));
  aoi022aa1n06x5               g089(.a(new_n178), .b(new_n183), .c(new_n172), .d(new_n184), .o1(new_n185));
  aoai13aa1n12x5               g090(.a(new_n185), .b(new_n179), .c(new_n182), .d(new_n147), .o1(new_n186));
  inv040aa1n06x5               g091(.a(new_n186), .o1(new_n187));
  xorc02aa1n02x5               g092(.a(\a[17] ), .b(\b[16] ), .out0(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n188), .b(new_n181), .c(new_n187), .out0(\s[17] ));
  inv040aa1d32x5               g094(.a(\a[18] ), .o1(new_n190));
  nand22aa1n03x5               g095(.a(new_n181), .b(new_n187), .o1(new_n191));
  nor042aa1n06x5               g096(.a(\b[16] ), .b(\a[17] ), .o1(new_n192));
  tech160nm_fiaoi012aa1n05x5   g097(.a(new_n192), .b(new_n191), .c(new_n188), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[17] ), .c(new_n190), .out0(\s[18] ));
  inv000aa1d42x5               g099(.a(\a[17] ), .o1(new_n195));
  xroi22aa1d04x5               g100(.a(new_n195), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n186), .c(new_n127), .d(new_n180), .o1(new_n197));
  inv000aa1d42x5               g102(.a(\b[17] ), .o1(new_n198));
  oaoi03aa1n12x5               g103(.a(new_n190), .b(new_n198), .c(new_n192), .o1(new_n199));
  nor042aa1n06x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nand02aa1d04x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n197), .c(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n02x5               g110(.a(new_n197), .b(new_n199), .o1(new_n206));
  nor042aa1n06x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nand22aa1n04x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  aoai13aa1n03x5               g114(.a(new_n209), .b(new_n200), .c(new_n206), .d(new_n203), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(new_n206), .b(new_n203), .o1(new_n211));
  nona22aa1n02x4               g116(.a(new_n211), .b(new_n209), .c(new_n200), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n212), .b(new_n210), .o1(\s[20] ));
  nona23aa1n09x5               g118(.a(new_n208), .b(new_n201), .c(new_n200), .d(new_n207), .out0(new_n214));
  oai012aa1n12x5               g119(.a(new_n208), .b(new_n207), .c(new_n200), .o1(new_n215));
  oai012aa1d24x5               g120(.a(new_n215), .b(new_n214), .c(new_n199), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  nano23aa1n06x5               g122(.a(new_n200), .b(new_n207), .c(new_n208), .d(new_n201), .out0(new_n218));
  nanp02aa1n02x5               g123(.a(new_n196), .b(new_n218), .o1(new_n219));
  aoai13aa1n06x5               g124(.a(new_n217), .b(new_n219), .c(new_n181), .d(new_n187), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[21] ), .b(\b[20] ), .out0(new_n223));
  xorc02aa1n02x5               g128(.a(\a[22] ), .b(\b[21] ), .out0(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  aoai13aa1n03x5               g130(.a(new_n225), .b(new_n222), .c(new_n220), .d(new_n223), .o1(new_n226));
  nand02aa1n04x5               g131(.a(new_n220), .b(new_n223), .o1(new_n227));
  nona22aa1n03x5               g132(.a(new_n227), .b(new_n225), .c(new_n222), .out0(new_n228));
  nanp02aa1n03x5               g133(.a(new_n228), .b(new_n226), .o1(\s[22] ));
  inv000aa1d42x5               g134(.a(\a[21] ), .o1(new_n230));
  inv040aa1d32x5               g135(.a(\a[22] ), .o1(new_n231));
  xroi22aa1d06x4               g136(.a(new_n230), .b(\b[20] ), .c(new_n231), .d(\b[21] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(\b[21] ), .o1(new_n233));
  oao003aa1n02x5               g138(.a(new_n231), .b(new_n233), .c(new_n222), .carry(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n216), .c(new_n232), .o1(new_n235));
  nanp03aa1n02x5               g140(.a(new_n232), .b(new_n196), .c(new_n218), .o1(new_n236));
  aoai13aa1n06x5               g141(.a(new_n235), .b(new_n236), .c(new_n181), .d(new_n187), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  xorc02aa1n12x5               g144(.a(\a[23] ), .b(\b[22] ), .out0(new_n240));
  xnrc02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .out0(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n239), .c(new_n237), .d(new_n240), .o1(new_n242));
  nand42aa1n03x5               g147(.a(new_n237), .b(new_n240), .o1(new_n243));
  nona22aa1n02x5               g148(.a(new_n243), .b(new_n241), .c(new_n239), .out0(new_n244));
  nanp02aa1n03x5               g149(.a(new_n244), .b(new_n242), .o1(\s[24] ));
  norb02aa1n02x5               g150(.a(new_n240), .b(new_n241), .out0(new_n246));
  nano22aa1n02x4               g151(.a(new_n219), .b(new_n246), .c(new_n232), .out0(new_n247));
  aoai13aa1n03x5               g152(.a(new_n247), .b(new_n186), .c(new_n127), .d(new_n180), .o1(new_n248));
  oao003aa1n02x5               g153(.a(new_n190), .b(new_n198), .c(new_n192), .carry(new_n249));
  inv040aa1n03x5               g154(.a(new_n215), .o1(new_n250));
  aoai13aa1n03x5               g155(.a(new_n232), .b(new_n250), .c(new_n218), .d(new_n249), .o1(new_n251));
  inv000aa1n02x5               g156(.a(new_n234), .o1(new_n252));
  inv020aa1n02x5               g157(.a(new_n246), .o1(new_n253));
  oai022aa1n02x5               g158(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n254));
  aob012aa1n02x5               g159(.a(new_n254), .b(\b[23] ), .c(\a[24] ), .out0(new_n255));
  aoai13aa1n02x7               g160(.a(new_n255), .b(new_n253), .c(new_n251), .d(new_n252), .o1(new_n256));
  nanb02aa1n02x5               g161(.a(new_n256), .b(new_n248), .out0(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  xorc02aa1n12x5               g165(.a(\a[26] ), .b(\b[25] ), .out0(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n259), .c(new_n257), .d(new_n260), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n260), .b(new_n256), .c(new_n191), .d(new_n247), .o1(new_n264));
  nona22aa1n03x5               g169(.a(new_n264), .b(new_n262), .c(new_n259), .out0(new_n265));
  nanp02aa1n02x5               g170(.a(new_n263), .b(new_n265), .o1(\s[26] ));
  and002aa1n12x5               g171(.a(new_n261), .b(new_n260), .o(new_n267));
  nano22aa1n03x7               g172(.a(new_n236), .b(new_n246), .c(new_n267), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n186), .c(new_n127), .d(new_n180), .o1(new_n269));
  nanp02aa1n03x5               g174(.a(new_n256), .b(new_n267), .o1(new_n270));
  oai022aa1n02x5               g175(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n271));
  aob012aa1n02x5               g176(.a(new_n271), .b(\b[25] ), .c(\a[26] ), .out0(new_n272));
  nand23aa1n04x5               g177(.a(new_n270), .b(new_n269), .c(new_n272), .o1(new_n273));
  xorb03aa1n02x5               g178(.a(new_n273), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  xorc02aa1n02x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n275), .c(new_n273), .d(new_n276), .o1(new_n278));
  aobi12aa1n12x5               g183(.a(new_n268), .b(new_n181), .c(new_n187), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n246), .b(new_n234), .c(new_n216), .d(new_n232), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n267), .o1(new_n281));
  aoai13aa1n06x5               g186(.a(new_n272), .b(new_n281), .c(new_n280), .d(new_n255), .o1(new_n282));
  oaih12aa1n02x5               g187(.a(new_n276), .b(new_n282), .c(new_n279), .o1(new_n283));
  nona22aa1n03x5               g188(.a(new_n283), .b(new_n277), .c(new_n275), .out0(new_n284));
  nanp02aa1n03x5               g189(.a(new_n278), .b(new_n284), .o1(\s[28] ));
  norb02aa1n02x5               g190(.a(new_n276), .b(new_n277), .out0(new_n286));
  oaih12aa1n02x5               g191(.a(new_n286), .b(new_n282), .c(new_n279), .o1(new_n287));
  inv000aa1n03x5               g192(.a(new_n275), .o1(new_n288));
  oaoi03aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .o1(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  nona22aa1n03x5               g195(.a(new_n287), .b(new_n289), .c(new_n290), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n290), .b(new_n289), .c(new_n273), .d(new_n286), .o1(new_n292));
  nanp02aa1n03x5               g197(.a(new_n292), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g199(.a(new_n276), .b(new_n290), .c(new_n277), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .carry(new_n296));
  oaoi03aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .o1(new_n297));
  tech160nm_fixorc02aa1n03p5x5 g202(.a(\a[30] ), .b(\b[29] ), .out0(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  aoai13aa1n02x7               g204(.a(new_n299), .b(new_n297), .c(new_n273), .d(new_n295), .o1(new_n300));
  oaih12aa1n02x5               g205(.a(new_n295), .b(new_n282), .c(new_n279), .o1(new_n301));
  nona22aa1n03x5               g206(.a(new_n301), .b(new_n297), .c(new_n299), .out0(new_n302));
  nanp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  nano23aa1n02x4               g208(.a(new_n290), .b(new_n277), .c(new_n298), .d(new_n276), .out0(new_n304));
  oaih12aa1n02x5               g209(.a(new_n304), .b(new_n282), .c(new_n279), .o1(new_n305));
  nanp02aa1n02x5               g210(.a(new_n297), .b(new_n298), .o1(new_n306));
  oai012aa1n02x5               g211(.a(new_n306), .b(\b[29] ), .c(\a[30] ), .o1(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  nona22aa1n03x5               g213(.a(new_n305), .b(new_n307), .c(new_n308), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n308), .b(new_n307), .c(new_n273), .d(new_n304), .o1(new_n310));
  nanp02aa1n03x5               g215(.a(new_n310), .b(new_n309), .o1(\s[31] ));
  oabi12aa1n02x5               g216(.a(new_n100), .b(new_n101), .c(new_n102), .out0(new_n312));
  xnrc02aa1n02x5               g217(.a(new_n312), .b(new_n109), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n312), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  nanp02aa1n02x5               g220(.a(new_n110), .b(new_n111), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g222(.a(new_n119), .b(new_n316), .c(new_n120), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g224(.a(new_n124), .b(new_n316), .c(new_n121), .o(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g226(.a(new_n114), .b(new_n320), .c(new_n115), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


