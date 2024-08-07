// Benchmark "adder" written by ABC on Thu Jul 18 10:36:59 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n176, new_n177, new_n178, new_n179, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n310, new_n311, new_n313,
    new_n314, new_n316, new_n317, new_n318, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1d32x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nand02aa1d08x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor002aa1d32x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nand02aa1d06x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nona23aa1d18x5               g007(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n103));
  tech160nm_fixnrc02aa1n02p5x5 g008(.a(\b[5] ), .b(\a[6] ), .out0(new_n104));
  tech160nm_fixnrc02aa1n04x5   g009(.a(\b[4] ), .b(\a[5] ), .out0(new_n105));
  nor043aa1n06x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  nor042aa1n04x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nand22aa1n12x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  nand22aa1n12x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  aoi012aa1n12x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  xnrc02aa1n12x5               g015(.a(\b[3] ), .b(\a[4] ), .out0(new_n111));
  norp02aa1n06x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nanb02aa1n12x5               g018(.a(new_n112), .b(new_n113), .out0(new_n114));
  inv000aa1d42x5               g019(.a(\a[3] ), .o1(new_n115));
  nanb02aa1n12x5               g020(.a(\b[2] ), .b(new_n115), .out0(new_n116));
  oao003aa1n03x5               g021(.a(\a[4] ), .b(\b[3] ), .c(new_n116), .carry(new_n117));
  oai013aa1n09x5               g022(.a(new_n117), .b(new_n111), .c(new_n110), .d(new_n114), .o1(new_n118));
  inv000aa1d42x5               g023(.a(new_n99), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n101), .b(new_n100), .o1(new_n120));
  inv040aa1d32x5               g025(.a(\b[5] ), .o1(new_n121));
  oai022aa1n04x5               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  oaib12aa1n06x5               g027(.a(new_n122), .b(new_n121), .c(\a[6] ), .out0(new_n123));
  oai112aa1n06x5               g028(.a(new_n119), .b(new_n120), .c(new_n103), .d(new_n123), .o1(new_n124));
  nanp02aa1n06x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n97), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n124), .c(new_n118), .d(new_n106), .o1(new_n127));
  nor002aa1d32x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanp02aa1n12x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  aoi012aa1d18x5               g036(.a(new_n124), .b(new_n118), .c(new_n106), .o1(new_n132));
  nona23aa1d18x5               g037(.a(new_n129), .b(new_n125), .c(new_n97), .d(new_n128), .out0(new_n133));
  oai012aa1n06x5               g038(.a(new_n129), .b(new_n128), .c(new_n97), .o1(new_n134));
  tech160nm_fioai012aa1n05x5   g039(.a(new_n134), .b(new_n132), .c(new_n133), .o1(new_n135));
  xorb03aa1n02x5               g040(.a(new_n135), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n20x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nand02aa1d16x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  aoi012aa1n02x5               g043(.a(new_n137), .b(new_n135), .c(new_n138), .o1(new_n139));
  xnrb03aa1n03x5               g044(.a(new_n139), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor022aa1n16x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1n16x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nona23aa1d18x5               g047(.a(new_n142), .b(new_n138), .c(new_n137), .d(new_n141), .out0(new_n143));
  ao0012aa1n03x7               g048(.a(new_n141), .b(new_n137), .c(new_n142), .o(new_n144));
  oabi12aa1n02x7               g049(.a(new_n144), .b(new_n143), .c(new_n134), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n133), .o1(new_n146));
  nano23aa1n06x5               g051(.a(new_n137), .b(new_n141), .c(new_n142), .d(new_n138), .out0(new_n147));
  nano22aa1n03x7               g052(.a(new_n132), .b(new_n146), .c(new_n147), .out0(new_n148));
  xnrc02aa1n12x5               g053(.a(\b[12] ), .b(\a[13] ), .out0(new_n149));
  oabi12aa1n02x5               g054(.a(new_n149), .b(new_n148), .c(new_n145), .out0(new_n150));
  norb03aa1n02x5               g055(.a(new_n149), .b(new_n148), .c(new_n145), .out0(new_n151));
  norb02aa1n02x5               g056(.a(new_n150), .b(new_n151), .out0(\s[13] ));
  orn002aa1n24x5               g057(.a(\a[13] ), .b(\b[12] ), .o(new_n153));
  tech160nm_fixnrc02aa1n05x5   g058(.a(\b[13] ), .b(\a[14] ), .out0(new_n154));
  xobna2aa1n03x5               g059(.a(new_n154), .b(new_n150), .c(new_n153), .out0(\s[14] ));
  nor042aa1n06x5               g060(.a(new_n154), .b(new_n149), .o1(new_n156));
  nano32aa1n03x7               g061(.a(new_n132), .b(new_n156), .c(new_n146), .d(new_n147), .out0(new_n157));
  inv040aa1n03x5               g062(.a(new_n134), .o1(new_n158));
  aoai13aa1n06x5               g063(.a(new_n156), .b(new_n144), .c(new_n147), .d(new_n158), .o1(new_n159));
  oao003aa1n12x5               g064(.a(\a[14] ), .b(\b[13] ), .c(new_n153), .carry(new_n160));
  nanp02aa1n02x5               g065(.a(new_n159), .b(new_n160), .o1(new_n161));
  nor042aa1n03x5               g066(.a(new_n157), .b(new_n161), .o1(new_n162));
  xorc02aa1n12x5               g067(.a(\a[15] ), .b(\b[14] ), .out0(new_n163));
  xnrc02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(\s[15] ));
  oaoi03aa1n03x5               g069(.a(\a[15] ), .b(\b[14] ), .c(new_n162), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  tech160nm_fixnrc02aa1n04x5   g071(.a(\b[15] ), .b(\a[16] ), .out0(new_n167));
  norb02aa1n12x5               g072(.a(new_n163), .b(new_n167), .out0(new_n168));
  nona23aa1d18x5               g073(.a(new_n168), .b(new_n156), .c(new_n133), .d(new_n143), .out0(new_n169));
  inv000aa1n02x5               g074(.a(new_n160), .o1(new_n170));
  aoai13aa1n04x5               g075(.a(new_n168), .b(new_n170), .c(new_n145), .d(new_n156), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n172));
  oab012aa1n02x4               g077(.a(new_n172), .b(\a[16] ), .c(\b[15] ), .out0(new_n173));
  oai112aa1n06x5               g078(.a(new_n171), .b(new_n173), .c(new_n132), .d(new_n169), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g080(.a(\a[18] ), .o1(new_n176));
  inv040aa1d32x5               g081(.a(\a[17] ), .o1(new_n177));
  inv040aa1n12x5               g082(.a(\b[16] ), .o1(new_n178));
  oaoi03aa1n03x5               g083(.a(new_n177), .b(new_n178), .c(new_n174), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[17] ), .c(new_n176), .out0(\s[18] ));
  inv000aa1n09x5               g085(.a(new_n132), .o1(new_n181));
  inv040aa1n06x5               g086(.a(new_n169), .o1(new_n182));
  inv000aa1n02x5               g087(.a(new_n168), .o1(new_n183));
  aoai13aa1n06x5               g088(.a(new_n173), .b(new_n183), .c(new_n159), .d(new_n160), .o1(new_n184));
  xroi22aa1d06x4               g089(.a(new_n177), .b(\b[16] ), .c(new_n176), .d(\b[17] ), .out0(new_n185));
  aoai13aa1n06x5               g090(.a(new_n185), .b(new_n184), .c(new_n181), .d(new_n182), .o1(new_n186));
  oai022aa1n09x5               g091(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n187));
  oaib12aa1n18x5               g092(.a(new_n187), .b(new_n176), .c(\b[17] ), .out0(new_n188));
  nor002aa1d32x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  nand42aa1d28x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(new_n189), .b(new_n190), .out0(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n192), .b(new_n186), .c(new_n188), .out0(\s[19] ));
  xnrc02aa1n02x5               g098(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g099(.a(new_n189), .o1(new_n195));
  aoi012aa1n02x5               g100(.a(new_n191), .b(new_n186), .c(new_n188), .o1(new_n196));
  nor002aa1d32x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nand42aa1d28x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  nano22aa1n02x4               g104(.a(new_n196), .b(new_n195), .c(new_n199), .out0(new_n200));
  nanp02aa1n02x5               g105(.a(new_n178), .b(new_n177), .o1(new_n201));
  oaoi03aa1n09x5               g106(.a(\a[18] ), .b(\b[17] ), .c(new_n201), .o1(new_n202));
  aoai13aa1n03x5               g107(.a(new_n192), .b(new_n202), .c(new_n174), .d(new_n185), .o1(new_n203));
  aoi012aa1n03x5               g108(.a(new_n199), .b(new_n203), .c(new_n195), .o1(new_n204));
  nor002aa1n02x5               g109(.a(new_n204), .b(new_n200), .o1(\s[20] ));
  nano23aa1n09x5               g110(.a(new_n189), .b(new_n197), .c(new_n198), .d(new_n190), .out0(new_n206));
  nand02aa1d04x5               g111(.a(new_n185), .b(new_n206), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n184), .c(new_n181), .d(new_n182), .o1(new_n209));
  nona23aa1d18x5               g114(.a(new_n198), .b(new_n190), .c(new_n189), .d(new_n197), .out0(new_n210));
  aoi012aa1d18x5               g115(.a(new_n197), .b(new_n189), .c(new_n198), .o1(new_n211));
  oai012aa1d24x5               g116(.a(new_n211), .b(new_n210), .c(new_n188), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  nor002aa1d32x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  nand02aa1n03x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n209), .c(new_n213), .out0(\s[21] ));
  inv000aa1d42x5               g122(.a(new_n214), .o1(new_n218));
  aobi12aa1n02x5               g123(.a(new_n216), .b(new_n209), .c(new_n213), .out0(new_n219));
  xnrc02aa1n12x5               g124(.a(\b[21] ), .b(\a[22] ), .out0(new_n220));
  nano22aa1n02x4               g125(.a(new_n219), .b(new_n218), .c(new_n220), .out0(new_n221));
  aoai13aa1n03x5               g126(.a(new_n216), .b(new_n212), .c(new_n174), .d(new_n208), .o1(new_n222));
  aoi012aa1n03x5               g127(.a(new_n220), .b(new_n222), .c(new_n218), .o1(new_n223));
  nor002aa1n02x5               g128(.a(new_n223), .b(new_n221), .o1(\s[22] ));
  nano22aa1n12x5               g129(.a(new_n220), .b(new_n218), .c(new_n215), .out0(new_n225));
  and003aa1n02x5               g130(.a(new_n185), .b(new_n225), .c(new_n206), .o(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n184), .c(new_n181), .d(new_n182), .o1(new_n227));
  oao003aa1n09x5               g132(.a(\a[22] ), .b(\b[21] ), .c(new_n218), .carry(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoi012aa1d18x5               g134(.a(new_n229), .b(new_n212), .c(new_n225), .o1(new_n230));
  xnrc02aa1n12x5               g135(.a(\b[22] ), .b(\a[23] ), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  xnbna2aa1n03x5               g137(.a(new_n232), .b(new_n227), .c(new_n230), .out0(\s[23] ));
  nor042aa1n09x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n231), .b(new_n227), .c(new_n230), .o1(new_n236));
  xnrc02aa1n12x5               g141(.a(\b[23] ), .b(\a[24] ), .out0(new_n237));
  nano22aa1n02x4               g142(.a(new_n236), .b(new_n235), .c(new_n237), .out0(new_n238));
  inv000aa1d42x5               g143(.a(new_n230), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n232), .b(new_n239), .c(new_n174), .d(new_n226), .o1(new_n240));
  aoi012aa1n03x5               g145(.a(new_n237), .b(new_n240), .c(new_n235), .o1(new_n241));
  nor002aa1n02x5               g146(.a(new_n241), .b(new_n238), .o1(\s[24] ));
  nor042aa1n03x5               g147(.a(new_n237), .b(new_n231), .o1(new_n243));
  nano22aa1n03x7               g148(.a(new_n207), .b(new_n225), .c(new_n243), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n184), .c(new_n181), .d(new_n182), .o1(new_n245));
  inv020aa1n03x5               g150(.a(new_n211), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n225), .b(new_n246), .c(new_n206), .d(new_n202), .o1(new_n247));
  inv040aa1n02x5               g152(.a(new_n243), .o1(new_n248));
  oao003aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .c(new_n235), .carry(new_n249));
  aoai13aa1n12x5               g154(.a(new_n249), .b(new_n248), .c(new_n247), .d(new_n228), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  xnrc02aa1n12x5               g156(.a(\b[24] ), .b(\a[25] ), .out0(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  xnbna2aa1n03x5               g158(.a(new_n253), .b(new_n245), .c(new_n251), .out0(\s[25] ));
  nor042aa1n03x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  aoi012aa1n02x5               g161(.a(new_n252), .b(new_n245), .c(new_n251), .o1(new_n257));
  xnrc02aa1n12x5               g162(.a(\b[25] ), .b(\a[26] ), .out0(new_n258));
  nano22aa1n02x4               g163(.a(new_n257), .b(new_n256), .c(new_n258), .out0(new_n259));
  aoai13aa1n03x5               g164(.a(new_n253), .b(new_n250), .c(new_n174), .d(new_n244), .o1(new_n260));
  aoi012aa1n03x5               g165(.a(new_n258), .b(new_n260), .c(new_n256), .o1(new_n261));
  nor002aa1n02x5               g166(.a(new_n261), .b(new_n259), .o1(\s[26] ));
  nor042aa1n09x5               g167(.a(new_n258), .b(new_n252), .o1(new_n263));
  nano32aa1n03x7               g168(.a(new_n207), .b(new_n263), .c(new_n225), .d(new_n243), .out0(new_n264));
  aoai13aa1n12x5               g169(.a(new_n264), .b(new_n184), .c(new_n181), .d(new_n182), .o1(new_n265));
  oao003aa1n02x5               g170(.a(\a[26] ), .b(\b[25] ), .c(new_n256), .carry(new_n266));
  aobi12aa1n12x5               g171(.a(new_n266), .b(new_n250), .c(new_n263), .out0(new_n267));
  xorc02aa1n12x5               g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  xnbna2aa1n06x5               g173(.a(new_n268), .b(new_n265), .c(new_n267), .out0(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  inv040aa1n03x5               g175(.a(new_n270), .o1(new_n271));
  aobi12aa1n06x5               g176(.a(new_n268), .b(new_n265), .c(new_n267), .out0(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  nano22aa1n03x7               g178(.a(new_n272), .b(new_n271), .c(new_n273), .out0(new_n274));
  aoai13aa1n03x5               g179(.a(new_n243), .b(new_n229), .c(new_n212), .d(new_n225), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n263), .o1(new_n276));
  aoai13aa1n04x5               g181(.a(new_n266), .b(new_n276), .c(new_n275), .d(new_n249), .o1(new_n277));
  aoai13aa1n02x7               g182(.a(new_n268), .b(new_n277), .c(new_n174), .d(new_n264), .o1(new_n278));
  aoi012aa1n03x5               g183(.a(new_n273), .b(new_n278), .c(new_n271), .o1(new_n279));
  norp02aa1n03x5               g184(.a(new_n279), .b(new_n274), .o1(\s[28] ));
  norb02aa1n02x5               g185(.a(new_n268), .b(new_n273), .out0(new_n281));
  aobi12aa1n06x5               g186(.a(new_n281), .b(new_n265), .c(new_n267), .out0(new_n282));
  oao003aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  nano22aa1n03x7               g189(.a(new_n282), .b(new_n283), .c(new_n284), .out0(new_n285));
  aoai13aa1n02x7               g190(.a(new_n281), .b(new_n277), .c(new_n174), .d(new_n264), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n284), .b(new_n286), .c(new_n283), .o1(new_n287));
  norp02aa1n03x5               g192(.a(new_n287), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n268), .b(new_n284), .c(new_n273), .out0(new_n290));
  aobi12aa1n06x5               g195(.a(new_n290), .b(new_n265), .c(new_n267), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  nano22aa1n03x7               g198(.a(new_n291), .b(new_n292), .c(new_n293), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n290), .b(new_n277), .c(new_n174), .d(new_n264), .o1(new_n295));
  aoi012aa1n03x5               g200(.a(new_n293), .b(new_n295), .c(new_n292), .o1(new_n296));
  norp02aa1n03x5               g201(.a(new_n296), .b(new_n294), .o1(\s[30] ));
  norb02aa1n02x5               g202(.a(new_n290), .b(new_n293), .out0(new_n298));
  aobi12aa1n06x5               g203(.a(new_n298), .b(new_n265), .c(new_n267), .out0(new_n299));
  oao003aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .c(new_n292), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  nano22aa1n03x7               g206(.a(new_n299), .b(new_n300), .c(new_n301), .out0(new_n302));
  aoai13aa1n02x7               g207(.a(new_n298), .b(new_n277), .c(new_n174), .d(new_n264), .o1(new_n303));
  aoi012aa1n03x5               g208(.a(new_n301), .b(new_n303), .c(new_n300), .o1(new_n304));
  norp02aa1n03x5               g209(.a(new_n304), .b(new_n302), .o1(\s[31] ));
  xnbna2aa1n03x5               g210(.a(new_n110), .b(new_n113), .c(new_n116), .out0(\s[3] ));
  oaoi03aa1n02x5               g211(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g213(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g214(.a(new_n105), .b(new_n118), .out0(new_n310));
  tech160nm_fioai012aa1n03p5x5 g215(.a(new_n310), .b(\b[4] ), .c(\a[5] ), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g217(.a(new_n104), .b(new_n311), .out0(new_n313));
  oaib12aa1n06x5               g218(.a(new_n313), .b(\a[6] ), .c(new_n121), .out0(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n02x5               g220(.a(new_n99), .b(new_n100), .out0(new_n316));
  inv000aa1d42x5               g221(.a(new_n316), .o1(new_n317));
  aoai13aa1n02x5               g222(.a(new_n317), .b(new_n101), .c(new_n314), .d(new_n102), .o1(new_n318));
  aoi112aa1n02x5               g223(.a(new_n317), .b(new_n101), .c(new_n314), .d(new_n102), .o1(new_n319));
  norb02aa1n02x7               g224(.a(new_n318), .b(new_n319), .out0(\s[8] ));
  xnbna2aa1n03x5               g225(.a(new_n132), .b(new_n125), .c(new_n98), .out0(\s[9] ));
endmodule


