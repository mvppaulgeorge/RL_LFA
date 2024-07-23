// Benchmark "adder" written by ABC on Wed Jul 17 20:58:02 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n139, new_n140, new_n141, new_n142, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n321, new_n323, new_n324, new_n325,
    new_n327, new_n329, new_n330, new_n331, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand02aa1n16x5               g001(.a(\b[3] ), .b(\a[4] ), .o1(new_n97));
  orn002aa1n12x5               g002(.a(\a[4] ), .b(\b[3] ), .o(new_n98));
  oai112aa1n06x5               g003(.a(new_n98), .b(new_n97), .c(\b[2] ), .d(\a[3] ), .o1(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand42aa1d28x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  norp02aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nona22aa1n09x5               g007(.a(new_n101), .b(new_n102), .c(new_n100), .out0(new_n103));
  nor002aa1n03x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1d28x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano22aa1n03x7               g010(.a(new_n104), .b(new_n101), .c(new_n105), .out0(new_n106));
  aoi012aa1n12x5               g011(.a(new_n99), .b(new_n106), .c(new_n103), .o1(new_n107));
  nor002aa1d32x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nor002aa1d32x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n09x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  norb03aa1n03x5               g015(.a(new_n110), .b(new_n108), .c(new_n109), .out0(new_n111));
  nand42aa1n06x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand02aa1n16x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor042aa1d18x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano22aa1n03x7               g019(.a(new_n114), .b(new_n112), .c(new_n113), .out0(new_n115));
  nand42aa1n20x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  oai112aa1n06x5               g021(.a(new_n97), .b(new_n116), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  nanb03aa1n09x5               g022(.a(new_n117), .b(new_n115), .c(new_n111), .out0(new_n118));
  inv040aa1n02x5               g023(.a(new_n114), .o1(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .o1(new_n120));
  nanb03aa1n02x5               g025(.a(new_n109), .b(new_n112), .c(new_n110), .out0(new_n121));
  norp02aa1n04x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  oai112aa1n04x5               g027(.a(new_n119), .b(new_n113), .c(new_n108), .d(new_n122), .o1(new_n123));
  oab012aa1n06x5               g028(.a(new_n120), .b(new_n123), .c(new_n121), .out0(new_n124));
  oai012aa1n18x5               g029(.a(new_n124), .b(new_n107), .c(new_n118), .o1(new_n125));
  norp02aa1n09x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  norb02aa1n03x4               g033(.a(new_n105), .b(new_n104), .out0(new_n129));
  oai112aa1n02x5               g034(.a(new_n129), .b(new_n101), .c(new_n102), .d(new_n100), .o1(new_n130));
  nanb02aa1n02x5               g035(.a(new_n99), .b(new_n130), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n109), .o1(new_n132));
  oai112aa1n02x5               g037(.a(new_n132), .b(new_n110), .c(\b[4] ), .d(\a[5] ), .o1(new_n133));
  nanb03aa1n02x5               g038(.a(new_n114), .b(new_n112), .c(new_n113), .out0(new_n134));
  norp03aa1n02x5               g039(.a(new_n133), .b(new_n134), .c(new_n117), .o1(new_n135));
  oabi12aa1n02x5               g040(.a(new_n120), .b(new_n121), .c(new_n123), .out0(new_n136));
  tech160nm_fiaoi012aa1n05x5   g041(.a(new_n136), .b(new_n131), .c(new_n135), .o1(new_n137));
  oaoi03aa1n02x5               g042(.a(\a[9] ), .b(\b[8] ), .c(new_n137), .o1(new_n138));
  norp02aa1n06x5               g043(.a(\b[9] ), .b(\a[10] ), .o1(new_n139));
  nanp02aa1n04x5               g044(.a(\b[9] ), .b(\a[10] ), .o1(new_n140));
  oaib12aa1n02x5               g045(.a(new_n138), .b(new_n139), .c(new_n140), .out0(new_n141));
  nona22aa1n02x4               g046(.a(new_n140), .b(new_n139), .c(new_n126), .out0(new_n142));
  aoai13aa1n02x5               g047(.a(new_n141), .b(new_n142), .c(new_n128), .d(new_n125), .o1(\s[10] ));
  nona23aa1n03x5               g048(.a(new_n140), .b(new_n127), .c(new_n126), .d(new_n139), .out0(new_n144));
  tech160nm_fioai012aa1n05x5   g049(.a(new_n140), .b(new_n139), .c(new_n126), .o1(new_n145));
  oai012aa1n02x5               g050(.a(new_n145), .b(new_n137), .c(new_n144), .o1(new_n146));
  xorb03aa1n02x5               g051(.a(new_n146), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g052(.a(\b[10] ), .b(\a[11] ), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  nand02aa1n04x5               g054(.a(\b[10] ), .b(\a[11] ), .o1(new_n150));
  nanp03aa1n03x5               g055(.a(new_n146), .b(new_n149), .c(new_n150), .o1(new_n151));
  norp02aa1n24x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nand02aa1n08x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  oai022aa1n02x5               g059(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n155));
  nanb03aa1n02x5               g060(.a(new_n155), .b(new_n151), .c(new_n153), .out0(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n154), .c(new_n149), .d(new_n151), .o1(\s[12] ));
  nona23aa1d18x5               g062(.a(new_n153), .b(new_n150), .c(new_n148), .d(new_n152), .out0(new_n158));
  norp02aa1n02x5               g063(.a(new_n158), .b(new_n144), .o1(new_n159));
  tech160nm_fioai012aa1n03p5x5 g064(.a(new_n153), .b(new_n152), .c(new_n148), .o1(new_n160));
  oai012aa1n12x5               g065(.a(new_n160), .b(new_n158), .c(new_n145), .o1(new_n161));
  nor042aa1n06x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand42aa1n20x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n161), .c(new_n125), .d(new_n159), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n164), .b(new_n161), .c(new_n125), .d(new_n159), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(\s[13] ));
  inv000aa1n02x5               g072(.a(new_n162), .o1(new_n168));
  nor042aa1n04x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand42aa1n16x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  nona23aa1n02x4               g076(.a(new_n165), .b(new_n170), .c(new_n169), .d(new_n162), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n171), .c(new_n168), .d(new_n165), .o1(\s[14] ));
  nano23aa1d12x5               g078(.a(new_n162), .b(new_n169), .c(new_n170), .d(new_n163), .out0(new_n174));
  oaoi03aa1n12x5               g079(.a(\a[14] ), .b(\b[13] ), .c(new_n168), .o1(new_n175));
  aoi012aa1n02x5               g080(.a(new_n175), .b(new_n161), .c(new_n174), .o1(new_n176));
  nona23aa1n06x5               g081(.a(new_n125), .b(new_n174), .c(new_n158), .d(new_n144), .out0(new_n177));
  nor002aa1n03x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nanp02aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n177), .c(new_n176), .out0(\s[15] ));
  tech160nm_finand02aa1n05x5   g086(.a(new_n177), .b(new_n176), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n178), .b(new_n182), .c(new_n180), .o1(new_n183));
  nor042aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanp02aa1n04x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(new_n186));
  oai022aa1n02x5               g091(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n187));
  aoi122aa1n02x5               g092(.a(new_n187), .b(\b[15] ), .c(\a[16] ), .d(new_n182), .e(new_n180), .o1(new_n188));
  oabi12aa1n02x5               g093(.a(new_n188), .b(new_n183), .c(new_n186), .out0(\s[16] ));
  nano23aa1n06x5               g094(.a(new_n178), .b(new_n184), .c(new_n185), .d(new_n179), .out0(new_n190));
  aoai13aa1n12x5               g095(.a(new_n190), .b(new_n175), .c(new_n161), .d(new_n174), .o1(new_n191));
  nano23aa1n06x5               g096(.a(new_n148), .b(new_n152), .c(new_n153), .d(new_n150), .out0(new_n192));
  nano32aa1n03x7               g097(.a(new_n144), .b(new_n190), .c(new_n192), .d(new_n174), .out0(new_n193));
  aoi022aa1n06x5               g098(.a(new_n125), .b(new_n193), .c(new_n185), .d(new_n187), .o1(new_n194));
  xnrc02aa1n02x5               g099(.a(\b[16] ), .b(\a[17] ), .out0(new_n195));
  xobna2aa1n03x5               g100(.a(new_n195), .b(new_n194), .c(new_n191), .out0(\s[17] ));
  inv000aa1d42x5               g101(.a(\a[17] ), .o1(new_n197));
  inv000aa1d42x5               g102(.a(\b[16] ), .o1(new_n198));
  nanp02aa1n12x5               g103(.a(new_n125), .b(new_n193), .o1(new_n199));
  oai012aa1n02x5               g104(.a(new_n185), .b(new_n184), .c(new_n178), .o1(new_n200));
  nanp03aa1d24x5               g105(.a(new_n199), .b(new_n191), .c(new_n200), .o1(new_n201));
  oaoi03aa1n02x5               g106(.a(new_n197), .b(new_n198), .c(new_n201), .o1(new_n202));
  xorc02aa1n02x5               g107(.a(\a[18] ), .b(\b[17] ), .out0(new_n203));
  oai022aa1d24x5               g108(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n204));
  aoi012aa1n02x5               g109(.a(new_n204), .b(\a[18] ), .c(\b[17] ), .o1(new_n205));
  aoai13aa1n02x5               g110(.a(new_n205), .b(new_n195), .c(new_n194), .d(new_n191), .o1(new_n206));
  oai012aa1n02x5               g111(.a(new_n206), .b(new_n202), .c(new_n203), .o1(\s[18] ));
  inv000aa1d42x5               g112(.a(\a[18] ), .o1(new_n208));
  xroi22aa1d04x5               g113(.a(new_n197), .b(\b[16] ), .c(new_n208), .d(\b[17] ), .out0(new_n209));
  aobi12aa1n02x5               g114(.a(new_n204), .b(\b[17] ), .c(\a[18] ), .out0(new_n210));
  xorc02aa1n12x5               g115(.a(\a[19] ), .b(\b[18] ), .out0(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n210), .c(new_n201), .d(new_n209), .o1(new_n212));
  aoi112aa1n02x5               g117(.a(new_n211), .b(new_n210), .c(new_n201), .d(new_n209), .o1(new_n213));
  norb02aa1n03x4               g118(.a(new_n212), .b(new_n213), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  orn002aa1n02x5               g120(.a(\a[19] ), .b(\b[18] ), .o(new_n216));
  xorc02aa1n02x5               g121(.a(\a[20] ), .b(\b[19] ), .out0(new_n217));
  nand42aa1n16x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  oai022aa1d18x5               g123(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n218), .b(new_n219), .out0(new_n220));
  nanp02aa1n06x5               g125(.a(new_n212), .b(new_n220), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n217), .c(new_n216), .d(new_n212), .o1(\s[20] ));
  nano32aa1n03x7               g127(.a(new_n195), .b(new_n217), .c(new_n203), .d(new_n211), .out0(new_n223));
  aoi022aa1d24x5               g128(.a(\b[18] ), .b(\a[19] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n224));
  aoai13aa1n12x5               g129(.a(new_n218), .b(new_n219), .c(new_n204), .d(new_n224), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  nor042aa1n06x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n226), .c(new_n201), .d(new_n223), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n229), .b(new_n226), .c(new_n201), .d(new_n223), .o1(new_n231));
  norb02aa1n03x4               g136(.a(new_n230), .b(new_n231), .out0(\s[21] ));
  inv000aa1d42x5               g137(.a(new_n227), .o1(new_n233));
  nor042aa1n02x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  nand42aa1n02x5               g139(.a(\b[21] ), .b(\a[22] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  norb03aa1n02x5               g141(.a(new_n235), .b(new_n227), .c(new_n234), .out0(new_n237));
  tech160nm_finand02aa1n05x5   g142(.a(new_n230), .b(new_n237), .o1(new_n238));
  aoai13aa1n03x5               g143(.a(new_n238), .b(new_n236), .c(new_n233), .d(new_n230), .o1(\s[22] ));
  nona23aa1n09x5               g144(.a(new_n235), .b(new_n228), .c(new_n227), .d(new_n234), .out0(new_n240));
  nano32aa1n02x4               g145(.a(new_n240), .b(new_n209), .c(new_n211), .d(new_n217), .out0(new_n241));
  tech160nm_fioai012aa1n03p5x5 g146(.a(new_n235), .b(new_n234), .c(new_n227), .o1(new_n242));
  oai012aa1n12x5               g147(.a(new_n242), .b(new_n225), .c(new_n240), .o1(new_n243));
  xorc02aa1n12x5               g148(.a(\a[23] ), .b(\b[22] ), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n243), .c(new_n201), .d(new_n241), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(new_n244), .b(new_n243), .c(new_n201), .d(new_n241), .o1(new_n246));
  norb02aa1n03x4               g151(.a(new_n245), .b(new_n246), .out0(\s[23] ));
  nor042aa1n03x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  tech160nm_fixorc02aa1n05x5   g154(.a(\a[24] ), .b(\b[23] ), .out0(new_n250));
  oai022aa1n02x5               g155(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n251), .b(\a[24] ), .c(\b[23] ), .o1(new_n252));
  nanp02aa1n06x5               g157(.a(new_n245), .b(new_n252), .o1(new_n253));
  aoai13aa1n03x5               g158(.a(new_n253), .b(new_n250), .c(new_n249), .d(new_n245), .o1(\s[24] ));
  inv000aa1n02x5               g159(.a(new_n223), .o1(new_n255));
  inv040aa1n03x5               g160(.a(new_n240), .o1(new_n256));
  nanp02aa1n02x5               g161(.a(new_n250), .b(new_n244), .o1(new_n257));
  inv000aa1n03x5               g162(.a(new_n257), .o1(new_n258));
  nano22aa1n02x4               g163(.a(new_n255), .b(new_n258), .c(new_n256), .out0(new_n259));
  nanp02aa1n02x5               g164(.a(new_n243), .b(new_n258), .o1(new_n260));
  oaoi03aa1n12x5               g165(.a(\a[24] ), .b(\b[23] ), .c(new_n249), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(new_n260), .b(new_n262), .o1(new_n263));
  tech160nm_fixorc02aa1n03p5x5 g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n263), .c(new_n201), .d(new_n259), .o1(new_n265));
  aoi112aa1n02x5               g170(.a(new_n264), .b(new_n263), .c(new_n201), .d(new_n259), .o1(new_n266));
  norb02aa1n03x4               g171(.a(new_n265), .b(new_n266), .out0(\s[25] ));
  nor042aa1n03x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  xorc02aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  oai022aa1n02x5               g175(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n271), .b(\a[26] ), .c(\b[25] ), .o1(new_n272));
  tech160nm_finand02aa1n05x5   g177(.a(new_n265), .b(new_n272), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n270), .c(new_n269), .d(new_n265), .o1(\s[26] ));
  and002aa1n02x5               g179(.a(new_n270), .b(new_n264), .o(new_n275));
  nano32aa1n03x7               g180(.a(new_n255), .b(new_n275), .c(new_n256), .d(new_n258), .out0(new_n276));
  inv020aa1n03x5               g181(.a(new_n276), .o1(new_n277));
  oaoi13aa1n04x5               g182(.a(new_n257), .b(new_n242), .c(new_n225), .d(new_n240), .o1(new_n278));
  oaoi03aa1n02x5               g183(.a(\a[26] ), .b(\b[25] ), .c(new_n269), .o1(new_n279));
  oaoi13aa1n03x5               g184(.a(new_n279), .b(new_n275), .c(new_n278), .d(new_n261), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n277), .c(new_n194), .d(new_n191), .o1(new_n281));
  xorb03aa1n03x5               g186(.a(new_n281), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  aoai13aa1n06x5               g189(.a(new_n275), .b(new_n261), .c(new_n243), .d(new_n258), .o1(new_n285));
  nanb02aa1n06x5               g190(.a(new_n279), .b(new_n285), .out0(new_n286));
  xorc02aa1n02x5               g191(.a(\a[27] ), .b(\b[26] ), .out0(new_n287));
  aoai13aa1n06x5               g192(.a(new_n287), .b(new_n286), .c(new_n201), .d(new_n276), .o1(new_n288));
  xorc02aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .out0(new_n289));
  oai022aa1n02x5               g194(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(\a[28] ), .c(\b[27] ), .o1(new_n291));
  tech160nm_finand02aa1n03p5x5 g196(.a(new_n288), .b(new_n291), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n289), .c(new_n284), .d(new_n288), .o1(\s[28] ));
  and002aa1n02x5               g198(.a(new_n289), .b(new_n287), .o(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n286), .c(new_n201), .d(new_n276), .o1(new_n295));
  inv000aa1d42x5               g200(.a(\b[27] ), .o1(new_n296));
  oaib12aa1n09x5               g201(.a(new_n290), .b(new_n296), .c(\a[28] ), .out0(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  tech160nm_fixorc02aa1n03p5x5 g203(.a(\a[29] ), .b(\b[28] ), .out0(new_n299));
  inv000aa1d42x5               g204(.a(new_n299), .o1(new_n300));
  nona22aa1n02x5               g205(.a(new_n295), .b(new_n298), .c(new_n300), .out0(new_n301));
  aoai13aa1n03x5               g206(.a(new_n300), .b(new_n298), .c(new_n281), .d(new_n294), .o1(new_n302));
  nanp02aa1n03x5               g207(.a(new_n302), .b(new_n301), .o1(\s[29] ));
  xorb03aa1n02x5               g208(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g209(.a(new_n300), .b(new_n287), .c(new_n289), .out0(new_n305));
  oaoi03aa1n02x5               g210(.a(\a[29] ), .b(\b[28] ), .c(new_n297), .o1(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[29] ), .b(\a[30] ), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n306), .c(new_n281), .d(new_n305), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n305), .b(new_n286), .c(new_n201), .d(new_n276), .o1(new_n309));
  nona22aa1n02x5               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  nanp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[30] ));
  nano32aa1n02x4               g216(.a(new_n307), .b(new_n299), .c(new_n289), .d(new_n287), .out0(new_n312));
  and002aa1n02x5               g217(.a(\b[29] ), .b(\a[30] ), .o(new_n313));
  oab012aa1n02x4               g218(.a(new_n313), .b(new_n306), .c(new_n307), .out0(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[30] ), .b(\a[31] ), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n314), .c(new_n281), .d(new_n312), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n312), .b(new_n286), .c(new_n201), .d(new_n276), .o1(new_n317));
  nona22aa1n02x5               g222(.a(new_n317), .b(new_n314), .c(new_n315), .out0(new_n318));
  nanp02aa1n03x5               g223(.a(new_n316), .b(new_n318), .o1(\s[31] ));
  xobna2aa1n03x5               g224(.a(new_n129), .b(new_n103), .c(new_n101), .out0(\s[3] ));
  aoi012aa1n02x5               g225(.a(new_n104), .b(new_n106), .c(new_n103), .o1(new_n321));
  aoai13aa1n02x5               g226(.a(new_n131), .b(new_n321), .c(new_n98), .d(new_n97), .o1(\s[4] ));
  nona23aa1n03x5               g227(.a(new_n116), .b(new_n97), .c(new_n107), .d(new_n108), .out0(new_n323));
  inv000aa1d42x5               g228(.a(new_n108), .o1(new_n324));
  aoi022aa1n02x5               g229(.a(new_n131), .b(new_n97), .c(new_n116), .d(new_n324), .o1(new_n325));
  norb02aa1n02x5               g230(.a(new_n323), .b(new_n325), .out0(\s[5] ));
  norb02aa1n02x5               g231(.a(new_n113), .b(new_n122), .out0(new_n327));
  xnbna2aa1n03x5               g232(.a(new_n327), .b(new_n323), .c(new_n324), .out0(\s[6] ));
  nanp03aa1n03x5               g233(.a(new_n323), .b(new_n324), .c(new_n327), .o1(new_n329));
  nanp02aa1n03x5               g234(.a(new_n329), .b(new_n115), .o1(new_n330));
  aoi022aa1n02x5               g235(.a(new_n329), .b(new_n113), .c(new_n112), .d(new_n119), .o1(new_n331));
  norb02aa1n02x5               g236(.a(new_n330), .b(new_n331), .out0(\s[7] ));
  norb02aa1n02x5               g237(.a(new_n110), .b(new_n109), .out0(new_n333));
  xnbna2aa1n03x5               g238(.a(new_n333), .b(new_n330), .c(new_n119), .out0(\s[8] ));
  xorb03aa1n02x5               g239(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


