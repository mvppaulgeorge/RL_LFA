// Benchmark "adder" written by ABC on Wed Jul 17 19:44:14 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n298, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n321, new_n323, new_n324, new_n325,
    new_n326, new_n328, new_n329, new_n331, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n10x5               g001(.a(\b[2] ), .b(\a[3] ), .o1(new_n97));
  nanp02aa1n04x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  nanb02aa1n09x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  oai112aa1n06x5               g004(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nanp02aa1n06x5               g006(.a(new_n100), .b(new_n101), .o1(new_n102));
  and002aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o(new_n103));
  nor022aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor043aa1n04x5               g009(.a(new_n103), .b(new_n104), .c(new_n97), .o1(new_n105));
  oaih12aa1n12x5               g010(.a(new_n105), .b(new_n102), .c(new_n99), .o1(new_n106));
  nand02aa1n10x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nor002aa1n06x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  nand42aa1n04x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nanb03aa1n02x5               g014(.a(new_n108), .b(new_n109), .c(new_n107), .out0(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .out0(new_n111));
  aoi022aa1n09x5               g016(.a(\b[4] ), .b(\a[5] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n112));
  oai122aa1n02x5               g017(.a(new_n112), .b(\a[6] ), .c(\b[5] ), .d(\a[5] ), .e(\b[4] ), .o1(new_n113));
  nona32aa1n09x5               g018(.a(new_n106), .b(new_n113), .c(new_n111), .d(new_n110), .out0(new_n114));
  nano22aa1n06x5               g019(.a(new_n108), .b(new_n107), .c(new_n109), .out0(new_n115));
  xorc02aa1n12x5               g020(.a(\a[8] ), .b(\b[7] ), .out0(new_n116));
  orn002aa1n24x5               g021(.a(\a[6] ), .b(\b[5] ), .o(new_n117));
  oai112aa1n06x5               g022(.a(new_n117), .b(new_n107), .c(\b[4] ), .d(\a[5] ), .o1(new_n118));
  aob012aa1n02x5               g023(.a(new_n108), .b(\b[7] ), .c(\a[8] ), .out0(new_n119));
  oai012aa1n02x5               g024(.a(new_n119), .b(\b[7] ), .c(\a[8] ), .o1(new_n120));
  aoi013aa1n06x4               g025(.a(new_n120), .b(new_n115), .c(new_n118), .d(new_n116), .o1(new_n121));
  xorc02aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  aobi12aa1n02x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .out0(new_n123));
  norp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  xorc02aa1n02x5               g029(.a(\a[10] ), .b(\b[9] ), .out0(new_n125));
  oabi12aa1n02x5               g030(.a(new_n125), .b(new_n123), .c(new_n124), .out0(new_n126));
  and002aa1n12x5               g031(.a(\b[9] ), .b(\a[10] ), .o(new_n127));
  oai022aa1d24x5               g032(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n128));
  nor042aa1n06x5               g033(.a(new_n128), .b(new_n127), .o1(new_n129));
  oaib12aa1n02x5               g034(.a(new_n126), .b(new_n123), .c(new_n129), .out0(\s[10] ));
  nand02aa1d06x5               g035(.a(new_n114), .b(new_n121), .o1(new_n131));
  nor002aa1d32x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  aoi022aa1d24x5               g037(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n133));
  norb02aa1n03x4               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  aoai13aa1n06x5               g039(.a(new_n134), .b(new_n128), .c(new_n131), .d(new_n122), .o1(new_n135));
  xnrc02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .out0(new_n136));
  oabi12aa1n02x5               g041(.a(new_n127), .b(new_n123), .c(new_n128), .out0(new_n137));
  aobi12aa1n02x5               g042(.a(new_n135), .b(new_n137), .c(new_n136), .out0(\s[11] ));
  inv000aa1d42x5               g043(.a(new_n132), .o1(new_n139));
  nor042aa1n06x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand02aa1d28x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n135), .c(new_n139), .out0(\s[12] ));
  nona23aa1d18x5               g048(.a(new_n133), .b(new_n141), .c(new_n140), .d(new_n132), .out0(new_n144));
  aoi112aa1n06x5               g049(.a(new_n144), .b(new_n128), .c(\a[9] ), .d(\b[8] ), .o1(new_n145));
  aoi012aa1n09x5               g050(.a(new_n140), .b(new_n132), .c(new_n141), .o1(new_n146));
  oai012aa1d24x5               g051(.a(new_n146), .b(new_n144), .c(new_n129), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  aob012aa1n03x5               g053(.a(new_n148), .b(new_n131), .c(new_n145), .out0(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  xnrc02aa1n06x5               g055(.a(\b[12] ), .b(\a[13] ), .out0(new_n151));
  nanb02aa1n03x5               g056(.a(new_n151), .b(new_n149), .out0(new_n152));
  xnrc02aa1n12x5               g057(.a(\b[13] ), .b(\a[14] ), .out0(new_n153));
  nor042aa1n06x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  tech160nm_fioai012aa1n03p5x5 g060(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .o1(new_n156));
  aboi22aa1n03x5               g061(.a(new_n153), .b(new_n156), .c(new_n152), .d(new_n155), .out0(\s[14] ));
  norp02aa1n02x5               g062(.a(new_n153), .b(new_n151), .o1(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n147), .c(new_n131), .d(new_n145), .o1(new_n159));
  inv000aa1n09x5               g064(.a(\a[14] ), .o1(new_n160));
  inv040aa1n04x5               g065(.a(\b[13] ), .o1(new_n161));
  tech160nm_fioaoi03aa1n03p5x5 g066(.a(new_n160), .b(new_n161), .c(new_n154), .o1(new_n162));
  nor002aa1d32x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nand22aa1n09x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  xnbna2aa1n03x5               g071(.a(new_n166), .b(new_n159), .c(new_n162), .out0(\s[15] ));
  aob012aa1n03x5               g072(.a(new_n166), .b(new_n159), .c(new_n162), .out0(new_n168));
  nor002aa1n12x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nand02aa1d06x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  aoib12aa1n02x5               g076(.a(new_n163), .b(new_n170), .c(new_n169), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n163), .o1(new_n173));
  aoai13aa1n02x7               g078(.a(new_n173), .b(new_n165), .c(new_n159), .d(new_n162), .o1(new_n174));
  aboi22aa1n03x5               g079(.a(new_n171), .b(new_n174), .c(new_n168), .d(new_n172), .out0(\s[16] ));
  nona23aa1n06x5               g080(.a(new_n170), .b(new_n164), .c(new_n163), .d(new_n169), .out0(new_n176));
  nor043aa1n03x5               g081(.a(new_n176), .b(new_n153), .c(new_n151), .o1(new_n177));
  nand02aa1d04x5               g082(.a(new_n145), .b(new_n177), .o1(new_n178));
  oai012aa1n02x5               g083(.a(new_n170), .b(new_n169), .c(new_n163), .o1(new_n179));
  tech160nm_fioai012aa1n04x5   g084(.a(new_n179), .b(new_n176), .c(new_n162), .o1(new_n180));
  aoi012aa1n12x5               g085(.a(new_n180), .b(new_n147), .c(new_n177), .o1(new_n181));
  aoai13aa1n12x5               g086(.a(new_n181), .b(new_n178), .c(new_n114), .d(new_n121), .o1(new_n182));
  xorb03aa1n03x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1d18x5               g088(.a(\b[16] ), .b(\a[17] ), .o1(new_n184));
  xorc02aa1n12x5               g089(.a(\a[17] ), .b(\b[16] ), .out0(new_n185));
  nor042aa1n06x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  nand42aa1d28x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  norb02aa1n06x5               g092(.a(new_n187), .b(new_n186), .out0(new_n188));
  aoi112aa1n02x5               g093(.a(new_n184), .b(new_n188), .c(new_n182), .d(new_n185), .o1(new_n189));
  aoai13aa1n02x7               g094(.a(new_n188), .b(new_n184), .c(new_n182), .d(new_n185), .o1(new_n190));
  norb02aa1n03x4               g095(.a(new_n190), .b(new_n189), .out0(\s[18] ));
  and002aa1n02x5               g096(.a(new_n185), .b(new_n188), .o(new_n192));
  aoi012aa1d24x5               g097(.a(new_n186), .b(new_n184), .c(new_n187), .o1(new_n193));
  inv000aa1d42x5               g098(.a(new_n193), .o1(new_n194));
  tech160nm_fixorc02aa1n03p5x5 g099(.a(\a[19] ), .b(\b[18] ), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n194), .c(new_n182), .d(new_n192), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n194), .c(new_n182), .d(new_n192), .o1(new_n197));
  norb02aa1n03x4               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  xorc02aa1n02x5               g104(.a(\a[20] ), .b(\b[19] ), .out0(new_n200));
  nor042aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norp02aa1n02x5               g106(.a(new_n200), .b(new_n201), .o1(new_n202));
  oai012aa1n06x5               g107(.a(new_n196), .b(\b[18] ), .c(\a[19] ), .o1(new_n203));
  aoi022aa1n02x7               g108(.a(new_n203), .b(new_n200), .c(new_n196), .d(new_n202), .o1(\s[20] ));
  tech160nm_fixnrc02aa1n04x5   g109(.a(\b[19] ), .b(\a[20] ), .out0(new_n205));
  nano32aa1n02x4               g110(.a(new_n205), .b(new_n195), .c(new_n185), .d(new_n188), .out0(new_n206));
  tech160nm_fixnrc02aa1n04x5   g111(.a(\b[18] ), .b(\a[19] ), .out0(new_n207));
  norp02aa1n02x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  aoi012aa1n09x5               g114(.a(new_n208), .b(new_n201), .c(new_n209), .o1(new_n210));
  oai013aa1n03x5               g115(.a(new_n210), .b(new_n207), .c(new_n205), .d(new_n193), .o1(new_n211));
  nor042aa1d18x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  nand02aa1n03x5               g117(.a(\b[20] ), .b(\a[21] ), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n211), .c(new_n182), .d(new_n206), .o1(new_n216));
  aoi112aa1n02x5               g121(.a(new_n215), .b(new_n211), .c(new_n182), .d(new_n206), .o1(new_n217));
  norb02aa1n03x4               g122(.a(new_n216), .b(new_n217), .out0(\s[21] ));
  nor002aa1n10x5               g123(.a(\b[21] ), .b(\a[22] ), .o1(new_n219));
  nand02aa1d28x5               g124(.a(\b[21] ), .b(\a[22] ), .o1(new_n220));
  nanb02aa1n02x5               g125(.a(new_n219), .b(new_n220), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoib12aa1n02x5               g127(.a(new_n212), .b(new_n220), .c(new_n219), .out0(new_n223));
  oai012aa1n06x5               g128(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .o1(new_n224));
  aoi022aa1n02x7               g129(.a(new_n224), .b(new_n222), .c(new_n216), .d(new_n223), .o1(\s[22] ));
  norp02aa1n02x5               g130(.a(new_n205), .b(new_n207), .o1(new_n226));
  nona23aa1n03x5               g131(.a(new_n220), .b(new_n213), .c(new_n212), .d(new_n219), .out0(new_n227));
  nano32aa1n02x4               g132(.a(new_n227), .b(new_n226), .c(new_n188), .d(new_n185), .out0(new_n228));
  nanb03aa1n03x5               g133(.a(new_n193), .b(new_n200), .c(new_n195), .out0(new_n229));
  ao0012aa1n12x5               g134(.a(new_n219), .b(new_n212), .c(new_n220), .o(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  aoai13aa1n02x5               g136(.a(new_n231), .b(new_n227), .c(new_n229), .d(new_n210), .o1(new_n232));
  xorc02aa1n12x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  aoai13aa1n09x5               g138(.a(new_n233), .b(new_n232), .c(new_n182), .d(new_n228), .o1(new_n234));
  nano23aa1n06x5               g139(.a(new_n212), .b(new_n219), .c(new_n220), .d(new_n213), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n233), .b(new_n230), .c(new_n211), .d(new_n235), .o1(new_n236));
  aobi12aa1n02x5               g141(.a(new_n236), .b(new_n182), .c(new_n228), .out0(new_n237));
  norb02aa1n03x4               g142(.a(new_n234), .b(new_n237), .out0(\s[23] ));
  xorc02aa1n12x5               g143(.a(\a[24] ), .b(\b[23] ), .out0(new_n239));
  nor002aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  norp02aa1n02x5               g145(.a(new_n239), .b(new_n240), .o1(new_n241));
  oai012aa1n06x5               g146(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .o1(new_n242));
  aoi022aa1n02x7               g147(.a(new_n242), .b(new_n239), .c(new_n234), .d(new_n241), .o1(\s[24] ));
  nanp03aa1n02x5               g148(.a(new_n235), .b(new_n233), .c(new_n239), .o1(new_n244));
  nano32aa1n02x5               g149(.a(new_n244), .b(new_n226), .c(new_n188), .d(new_n185), .out0(new_n245));
  and002aa1n02x5               g150(.a(new_n182), .b(new_n245), .o(new_n246));
  aob012aa1n02x5               g151(.a(new_n240), .b(\b[23] ), .c(\a[24] ), .out0(new_n247));
  oai012aa1n02x5               g152(.a(new_n247), .b(\b[23] ), .c(\a[24] ), .o1(new_n248));
  aoi013aa1n06x4               g153(.a(new_n248), .b(new_n230), .c(new_n233), .d(new_n239), .o1(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n244), .c(new_n229), .d(new_n210), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[25] ), .b(\b[24] ), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n250), .c(new_n182), .d(new_n245), .o1(new_n252));
  nano22aa1n02x5               g157(.a(new_n227), .b(new_n233), .c(new_n239), .out0(new_n253));
  nanp02aa1n02x5               g158(.a(new_n211), .b(new_n253), .o1(new_n254));
  nanb03aa1n02x5               g159(.a(new_n251), .b(new_n254), .c(new_n249), .out0(new_n255));
  oa0012aa1n03x5               g160(.a(new_n252), .b(new_n246), .c(new_n255), .o(\s[25] ));
  xorc02aa1n02x5               g161(.a(\a[26] ), .b(\b[25] ), .out0(new_n257));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  norp02aa1n02x5               g163(.a(new_n257), .b(new_n258), .o1(new_n259));
  inv040aa1d30x5               g164(.a(\a[25] ), .o1(new_n260));
  oaib12aa1n06x5               g165(.a(new_n252), .b(\b[24] ), .c(new_n260), .out0(new_n261));
  aoi022aa1n02x7               g166(.a(new_n261), .b(new_n257), .c(new_n252), .d(new_n259), .o1(\s[26] ));
  nano23aa1n02x4               g167(.a(new_n163), .b(new_n169), .c(new_n170), .d(new_n164), .out0(new_n263));
  nona22aa1n02x4               g168(.a(new_n263), .b(new_n153), .c(new_n151), .out0(new_n264));
  norb02aa1n02x5               g169(.a(new_n145), .b(new_n264), .out0(new_n265));
  oai112aa1n02x5               g170(.a(new_n134), .b(new_n142), .c(new_n128), .d(new_n127), .o1(new_n266));
  oao003aa1n02x5               g171(.a(new_n160), .b(new_n161), .c(new_n154), .carry(new_n267));
  aobi12aa1n02x5               g172(.a(new_n179), .b(new_n263), .c(new_n267), .out0(new_n268));
  aoai13aa1n02x5               g173(.a(new_n268), .b(new_n264), .c(new_n266), .d(new_n146), .o1(new_n269));
  inv040aa1d32x5               g174(.a(\a[26] ), .o1(new_n270));
  xroi22aa1d06x4               g175(.a(new_n260), .b(\b[24] ), .c(new_n270), .d(\b[25] ), .out0(new_n271));
  nano32aa1n03x7               g176(.a(new_n244), .b(new_n192), .c(new_n271), .d(new_n226), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n269), .c(new_n131), .d(new_n265), .o1(new_n273));
  oai022aa1n02x5               g178(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n274));
  oaib12aa1n02x5               g179(.a(new_n274), .b(new_n270), .c(\b[25] ), .out0(new_n275));
  aobi12aa1n06x5               g180(.a(new_n275), .b(new_n250), .c(new_n271), .out0(new_n276));
  xorc02aa1n02x5               g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n273), .c(new_n276), .out0(\s[27] ));
  inv000aa1n02x5               g183(.a(new_n271), .o1(new_n279));
  aoai13aa1n03x5               g184(.a(new_n275), .b(new_n279), .c(new_n254), .d(new_n249), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n277), .b(new_n280), .c(new_n182), .d(new_n272), .o1(new_n281));
  xorc02aa1n02x5               g186(.a(\a[28] ), .b(\b[27] ), .out0(new_n282));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  norp02aa1n02x5               g188(.a(new_n282), .b(new_n283), .o1(new_n284));
  inv000aa1d42x5               g189(.a(\a[27] ), .o1(new_n285));
  oaib12aa1n06x5               g190(.a(new_n281), .b(\b[26] ), .c(new_n285), .out0(new_n286));
  aoi022aa1n02x7               g191(.a(new_n286), .b(new_n282), .c(new_n281), .d(new_n284), .o1(\s[28] ));
  inv000aa1d42x5               g192(.a(\a[28] ), .o1(new_n288));
  xroi22aa1d06x4               g193(.a(new_n285), .b(\b[26] ), .c(new_n288), .d(\b[27] ), .out0(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n280), .c(new_n182), .d(new_n272), .o1(new_n290));
  inv000aa1d42x5               g195(.a(\b[27] ), .o1(new_n291));
  oao003aa1n09x5               g196(.a(new_n288), .b(new_n291), .c(new_n283), .carry(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  nanp02aa1n03x5               g198(.a(new_n290), .b(new_n293), .o1(new_n294));
  tech160nm_fixorc02aa1n02p5x5 g199(.a(\a[29] ), .b(\b[28] ), .out0(new_n295));
  norp02aa1n02x5               g200(.a(new_n292), .b(new_n295), .o1(new_n296));
  aoi022aa1n02x7               g201(.a(new_n294), .b(new_n295), .c(new_n290), .d(new_n296), .o1(\s[29] ));
  nanp02aa1n02x5               g202(.a(\b[0] ), .b(\a[1] ), .o1(new_n298));
  xorb03aa1n02x5               g203(.a(new_n298), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g204(.a(new_n277), .b(new_n295), .c(new_n282), .o(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n280), .c(new_n182), .d(new_n272), .o1(new_n301));
  inv000aa1d42x5               g206(.a(\a[29] ), .o1(new_n302));
  inv000aa1d42x5               g207(.a(\b[28] ), .o1(new_n303));
  oaoi03aa1n02x5               g208(.a(new_n302), .b(new_n303), .c(new_n292), .o1(new_n304));
  nanp02aa1n03x5               g209(.a(new_n301), .b(new_n304), .o1(new_n305));
  xorc02aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .out0(new_n306));
  oabi12aa1n02x5               g211(.a(new_n306), .b(\a[29] ), .c(\b[28] ), .out0(new_n307));
  oaoi13aa1n02x5               g212(.a(new_n307), .b(new_n292), .c(new_n302), .d(new_n303), .o1(new_n308));
  aoi022aa1n02x7               g213(.a(new_n305), .b(new_n306), .c(new_n301), .d(new_n308), .o1(\s[30] ));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  and003aa1n02x5               g215(.a(new_n289), .b(new_n306), .c(new_n295), .o(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n280), .c(new_n182), .d(new_n272), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n313));
  aoi012aa1n03x5               g218(.a(new_n310), .b(new_n312), .c(new_n313), .o1(new_n314));
  inv000aa1n02x5               g219(.a(new_n311), .o1(new_n315));
  aoi012aa1n02x5               g220(.a(new_n315), .b(new_n273), .c(new_n276), .o1(new_n316));
  nano22aa1n03x5               g221(.a(new_n316), .b(new_n310), .c(new_n313), .out0(new_n317));
  nor002aa1n02x5               g222(.a(new_n314), .b(new_n317), .o1(\s[31] ));
  xnbna2aa1n03x5               g223(.a(new_n99), .b(new_n100), .c(new_n101), .out0(\s[3] ));
  xnrc02aa1n02x5               g224(.a(\b[3] ), .b(\a[4] ), .out0(new_n320));
  aoi013aa1n02x4               g225(.a(new_n97), .b(new_n100), .c(new_n101), .d(new_n98), .o1(new_n321));
  oaib12aa1n02x5               g226(.a(new_n106), .b(new_n321), .c(new_n320), .out0(\s[4] ));
  orn002aa1n02x5               g227(.a(\a[5] ), .b(\b[4] ), .o(new_n323));
  and002aa1n02x5               g228(.a(new_n112), .b(new_n323), .o(new_n324));
  nanb02aa1n02x5               g229(.a(new_n103), .b(new_n106), .out0(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[4] ), .b(\a[5] ), .out0(new_n326));
  aoi022aa1n02x5               g231(.a(new_n325), .b(new_n326), .c(new_n324), .d(new_n106), .o1(\s[5] ));
  aobi12aa1n02x5               g232(.a(new_n323), .b(new_n106), .c(new_n112), .out0(new_n328));
  tech160nm_fiao0012aa1n02p5x5 g233(.a(new_n118), .b(new_n106), .c(new_n324), .o(new_n329));
  aoai13aa1n02x5               g234(.a(new_n329), .b(new_n328), .c(new_n107), .d(new_n117), .o1(\s[6] ));
  norb02aa1n02x5               g235(.a(new_n109), .b(new_n108), .out0(new_n331));
  xobna2aa1n03x5               g236(.a(new_n331), .b(new_n329), .c(new_n107), .out0(\s[7] ));
  aoi012aa1n02x5               g237(.a(new_n108), .b(new_n329), .c(new_n115), .o1(new_n333));
  xnrc02aa1n02x5               g238(.a(new_n333), .b(new_n116), .out0(\s[8] ));
  xnbna2aa1n03x5               g239(.a(new_n122), .b(new_n114), .c(new_n121), .out0(\s[9] ));
endmodule


