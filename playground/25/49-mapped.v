// Benchmark "adder" written by ABC on Thu Jul 18 01:14:03 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n311, new_n314, new_n315, new_n316, new_n319, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv030aa1n06x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1d32x5               g003(.a(\b[5] ), .b(\a[6] ), .o1(new_n99));
  nand42aa1n16x5               g004(.a(\b[5] ), .b(\a[6] ), .o1(new_n100));
  nor022aa1n16x5               g005(.a(\b[4] ), .b(\a[5] ), .o1(new_n101));
  tech160nm_finand02aa1n03p5x5 g006(.a(\b[4] ), .b(\a[5] ), .o1(new_n102));
  nona23aa1n09x5               g007(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n103));
  xnrc02aa1n12x5               g008(.a(\b[7] ), .b(\a[8] ), .out0(new_n104));
  xnrc02aa1n12x5               g009(.a(\b[6] ), .b(\a[7] ), .out0(new_n105));
  nor043aa1n04x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  nor042aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nand02aa1d04x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  nand02aa1d04x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  aoi012aa1n12x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nand02aa1d10x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  tech160nm_fiaoi012aa1n03p5x5 g020(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n116));
  oai012aa1n18x5               g021(.a(new_n116), .b(new_n115), .c(new_n110), .o1(new_n117));
  inv040aa1d28x5               g022(.a(\a[5] ), .o1(new_n118));
  inv040aa1n18x5               g023(.a(\b[4] ), .o1(new_n119));
  aoai13aa1n04x5               g024(.a(new_n100), .b(new_n99), .c(new_n118), .d(new_n119), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[8] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[7] ), .o1(new_n122));
  nor002aa1n02x5               g027(.a(\b[6] ), .b(\a[7] ), .o1(new_n123));
  oaoi03aa1n09x5               g028(.a(new_n121), .b(new_n122), .c(new_n123), .o1(new_n124));
  oai013aa1n06x5               g029(.a(new_n124), .b(new_n105), .c(new_n104), .d(new_n120), .o1(new_n125));
  nand02aa1n08x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n97), .out0(new_n127));
  aoai13aa1n04x5               g032(.a(new_n127), .b(new_n125), .c(new_n117), .d(new_n106), .o1(new_n128));
  nor042aa1n12x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nanp02aa1n06x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g037(.a(new_n129), .o1(new_n133));
  inv000aa1n02x5               g038(.a(new_n131), .o1(new_n134));
  aoai13aa1n06x5               g039(.a(new_n133), .b(new_n134), .c(new_n128), .d(new_n98), .o1(new_n135));
  xorb03aa1n02x5               g040(.a(new_n135), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d24x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nand42aa1d28x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nor002aa1n16x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand42aa1n16x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n141), .o1(new_n142));
  aoi112aa1n03x5               g047(.a(new_n142), .b(new_n137), .c(new_n135), .d(new_n138), .o1(new_n143));
  aoai13aa1n04x5               g048(.a(new_n142), .b(new_n137), .c(new_n135), .d(new_n138), .o1(new_n144));
  norb02aa1n03x4               g049(.a(new_n144), .b(new_n143), .out0(\s[12] ));
  nano23aa1d15x5               g050(.a(new_n137), .b(new_n139), .c(new_n140), .d(new_n138), .out0(new_n146));
  nano23aa1d15x5               g051(.a(new_n97), .b(new_n129), .c(new_n130), .d(new_n126), .out0(new_n147));
  nand22aa1n12x5               g052(.a(new_n147), .b(new_n146), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n125), .c(new_n117), .d(new_n106), .o1(new_n150));
  oa0012aa1n09x5               g055(.a(new_n140), .b(new_n139), .c(new_n137), .o(new_n151));
  tech160nm_fioaoi03aa1n02p5x5 g056(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n151), .b(new_n146), .c(new_n152), .o1(new_n153));
  nor042aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanp02aa1n04x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n03x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n150), .c(new_n153), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(\a[13] ), .o1(new_n158));
  inv000aa1d42x5               g063(.a(\b[12] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(new_n150), .b(new_n153), .o1(new_n160));
  oaoi03aa1n02x5               g065(.a(new_n158), .b(new_n159), .c(new_n160), .o1(new_n161));
  nor042aa1n04x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nand42aa1n08x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  norb02aa1n09x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  xnrc02aa1n02x5               g069(.a(new_n161), .b(new_n164), .out0(\s[14] ));
  nona23aa1n02x4               g070(.a(new_n163), .b(new_n155), .c(new_n154), .d(new_n162), .out0(new_n166));
  aoai13aa1n04x5               g071(.a(new_n163), .b(new_n162), .c(new_n158), .d(new_n159), .o1(new_n167));
  aoai13aa1n04x5               g072(.a(new_n167), .b(new_n166), .c(new_n150), .d(new_n153), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nand02aa1n08x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  norb02aa1n03x4               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  nor002aa1n16x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n06x4               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  aoi112aa1n02x5               g080(.a(new_n170), .b(new_n175), .c(new_n168), .d(new_n172), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n175), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(\s[16] ));
  nano23aa1n06x5               g083(.a(new_n170), .b(new_n173), .c(new_n174), .d(new_n171), .out0(new_n179));
  nano32aa1d12x5               g084(.a(new_n148), .b(new_n179), .c(new_n156), .d(new_n164), .out0(new_n180));
  aoai13aa1n12x5               g085(.a(new_n180), .b(new_n125), .c(new_n117), .d(new_n106), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n173), .o1(new_n182));
  nanb03aa1n03x5               g087(.a(new_n167), .b(new_n175), .c(new_n172), .out0(new_n183));
  aoi112aa1n03x5               g088(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n184), .o1(new_n185));
  inv040aa1n03x5               g090(.a(new_n151), .o1(new_n186));
  nanp02aa1n03x5               g091(.a(new_n146), .b(new_n152), .o1(new_n187));
  nanp03aa1n02x5               g092(.a(new_n179), .b(new_n156), .c(new_n164), .o1(new_n188));
  tech160nm_fiaoi012aa1n04x5   g093(.a(new_n188), .b(new_n187), .c(new_n186), .o1(new_n189));
  nano32aa1d12x5               g094(.a(new_n189), .b(new_n185), .c(new_n183), .d(new_n182), .out0(new_n190));
  nanp02aa1n09x5               g095(.a(new_n190), .b(new_n181), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g097(.a(\a[18] ), .o1(new_n193));
  inv040aa1d30x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[16] ), .o1(new_n195));
  oaoi03aa1n03x5               g100(.a(new_n194), .b(new_n195), .c(new_n191), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n193), .out0(\s[18] ));
  xroi22aa1d06x4               g102(.a(new_n194), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  oai022aa1n02x5               g104(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n200));
  oaib12aa1n02x5               g105(.a(new_n200), .b(new_n193), .c(\b[17] ), .out0(new_n201));
  aoai13aa1n04x5               g106(.a(new_n201), .b(new_n199), .c(new_n190), .d(new_n181), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nand42aa1n08x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  norp02aa1n09x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nand42aa1n08x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  aoi112aa1n02x5               g115(.a(new_n205), .b(new_n210), .c(new_n202), .d(new_n206), .o1(new_n211));
  aoai13aa1n03x5               g116(.a(new_n210), .b(new_n205), .c(new_n202), .d(new_n206), .o1(new_n212));
  norb02aa1n02x7               g117(.a(new_n212), .b(new_n211), .out0(\s[20] ));
  nano23aa1d15x5               g118(.a(new_n205), .b(new_n207), .c(new_n208), .d(new_n206), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(new_n198), .b(new_n214), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(new_n195), .b(new_n194), .o1(new_n216));
  oaoi03aa1n02x5               g121(.a(\a[18] ), .b(\b[17] ), .c(new_n216), .o1(new_n217));
  tech160nm_fiao0012aa1n02p5x5 g122(.a(new_n207), .b(new_n205), .c(new_n208), .o(new_n218));
  tech160nm_fiaoi012aa1n03p5x5 g123(.a(new_n218), .b(new_n214), .c(new_n217), .o1(new_n219));
  aoai13aa1n04x5               g124(.a(new_n219), .b(new_n215), .c(new_n190), .d(new_n181), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xnrc02aa1n12x5               g127(.a(\b[20] ), .b(\a[21] ), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoi112aa1n02x7               g131(.a(new_n222), .b(new_n226), .c(new_n220), .d(new_n224), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n226), .b(new_n222), .c(new_n220), .d(new_n224), .o1(new_n228));
  norb02aa1n02x7               g133(.a(new_n228), .b(new_n227), .out0(\s[22] ));
  norp02aa1n06x5               g134(.a(new_n225), .b(new_n223), .o1(new_n230));
  nand23aa1n06x5               g135(.a(new_n198), .b(new_n230), .c(new_n214), .o1(new_n231));
  nona23aa1n03x5               g136(.a(new_n208), .b(new_n206), .c(new_n205), .d(new_n207), .out0(new_n232));
  oabi12aa1n03x5               g137(.a(new_n218), .b(new_n232), .c(new_n201), .out0(new_n233));
  orn002aa1n02x5               g138(.a(\a[21] ), .b(\b[20] ), .o(new_n234));
  oaoi03aa1n02x5               g139(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n233), .c(new_n230), .o1(new_n236));
  aoai13aa1n04x5               g141(.a(new_n236), .b(new_n231), .c(new_n190), .d(new_n181), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  nand42aa1n10x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  nor042aa1n03x5               g145(.a(\b[23] ), .b(\a[24] ), .o1(new_n241));
  tech160nm_finand02aa1n05x5   g146(.a(\b[23] ), .b(\a[24] ), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(new_n243));
  aoi112aa1n02x5               g148(.a(new_n239), .b(new_n243), .c(new_n237), .d(new_n240), .o1(new_n244));
  aoai13aa1n03x5               g149(.a(new_n243), .b(new_n239), .c(new_n237), .d(new_n240), .o1(new_n245));
  norb02aa1n02x7               g150(.a(new_n245), .b(new_n244), .out0(\s[24] ));
  nona23aa1n03x5               g151(.a(new_n242), .b(new_n240), .c(new_n239), .d(new_n241), .out0(new_n247));
  nona23aa1n03x5               g152(.a(new_n198), .b(new_n230), .c(new_n247), .d(new_n232), .out0(new_n248));
  aoi112aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n249));
  inv000aa1n02x5               g154(.a(new_n249), .o1(new_n250));
  nano23aa1n03x5               g155(.a(new_n239), .b(new_n241), .c(new_n242), .d(new_n240), .out0(new_n251));
  nano22aa1n03x7               g156(.a(new_n219), .b(new_n230), .c(new_n251), .out0(new_n252));
  aoi012aa1n03x5               g157(.a(new_n241), .b(new_n251), .c(new_n235), .o1(new_n253));
  nano22aa1n03x7               g158(.a(new_n252), .b(new_n250), .c(new_n253), .out0(new_n254));
  aoai13aa1n04x5               g159(.a(new_n254), .b(new_n248), .c(new_n190), .d(new_n181), .o1(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  xorc02aa1n03x5               g162(.a(\a[25] ), .b(\b[24] ), .out0(new_n258));
  tech160nm_fixorc02aa1n05x5   g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  aoi112aa1n02x5               g164(.a(new_n257), .b(new_n259), .c(new_n255), .d(new_n258), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n259), .b(new_n257), .c(new_n255), .d(new_n258), .o1(new_n261));
  norb02aa1n02x7               g166(.a(new_n261), .b(new_n260), .out0(\s[26] ));
  and002aa1n06x5               g167(.a(new_n259), .b(new_n258), .o(new_n263));
  nano22aa1n03x7               g168(.a(new_n231), .b(new_n263), .c(new_n251), .out0(new_n264));
  nand02aa1d06x5               g169(.a(new_n191), .b(new_n264), .o1(new_n265));
  nona32aa1n03x5               g170(.a(new_n233), .b(new_n247), .c(new_n225), .d(new_n223), .out0(new_n266));
  nand23aa1n03x5               g171(.a(new_n266), .b(new_n250), .c(new_n253), .o1(new_n267));
  orn002aa1n02x5               g172(.a(\a[25] ), .b(\b[24] ), .o(new_n268));
  oao003aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .carry(new_n269));
  aobi12aa1n09x5               g174(.a(new_n269), .b(new_n267), .c(new_n263), .out0(new_n270));
  xorc02aa1n12x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnbna2aa1n06x5               g176(.a(new_n271), .b(new_n265), .c(new_n270), .out0(\s[27] ));
  nor042aa1n03x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  aobi12aa1n06x5               g179(.a(new_n271), .b(new_n265), .c(new_n270), .out0(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  nano22aa1n02x4               g181(.a(new_n275), .b(new_n274), .c(new_n276), .out0(new_n277));
  aobi12aa1n06x5               g182(.a(new_n264), .b(new_n190), .c(new_n181), .out0(new_n278));
  inv000aa1d42x5               g183(.a(new_n263), .o1(new_n279));
  tech160nm_fioai012aa1n04x5   g184(.a(new_n269), .b(new_n254), .c(new_n279), .o1(new_n280));
  oaih12aa1n02x5               g185(.a(new_n271), .b(new_n280), .c(new_n278), .o1(new_n281));
  tech160nm_fiaoi012aa1n02p5x5 g186(.a(new_n276), .b(new_n281), .c(new_n274), .o1(new_n282));
  norp02aa1n03x5               g187(.a(new_n282), .b(new_n277), .o1(\s[28] ));
  norb02aa1n02x5               g188(.a(new_n271), .b(new_n276), .out0(new_n284));
  oaih12aa1n02x5               g189(.a(new_n284), .b(new_n280), .c(new_n278), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .c(new_n274), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[28] ), .b(\a[29] ), .out0(new_n287));
  tech160nm_fiaoi012aa1n02p5x5 g192(.a(new_n287), .b(new_n285), .c(new_n286), .o1(new_n288));
  aobi12aa1n06x5               g193(.a(new_n284), .b(new_n265), .c(new_n270), .out0(new_n289));
  nano22aa1n02x4               g194(.a(new_n289), .b(new_n286), .c(new_n287), .out0(new_n290));
  norp02aa1n03x5               g195(.a(new_n288), .b(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g197(.a(new_n271), .b(new_n287), .c(new_n276), .out0(new_n293));
  oaih12aa1n02x5               g198(.a(new_n293), .b(new_n280), .c(new_n278), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .out0(new_n296));
  tech160nm_fiaoi012aa1n03p5x5 g201(.a(new_n296), .b(new_n294), .c(new_n295), .o1(new_n297));
  aobi12aa1n06x5               g202(.a(new_n293), .b(new_n265), .c(new_n270), .out0(new_n298));
  nano22aa1n02x4               g203(.a(new_n298), .b(new_n295), .c(new_n296), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[30] ));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  norb02aa1n02x5               g206(.a(new_n293), .b(new_n296), .out0(new_n302));
  oaih12aa1n02x5               g207(.a(new_n302), .b(new_n280), .c(new_n278), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n295), .carry(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n301), .b(new_n303), .c(new_n304), .o1(new_n305));
  aobi12aa1n06x5               g210(.a(new_n302), .b(new_n265), .c(new_n270), .out0(new_n306));
  nano22aa1n02x4               g211(.a(new_n306), .b(new_n301), .c(new_n304), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n305), .b(new_n307), .o1(\s[31] ));
  xnrb03aa1n02x5               g213(.a(new_n110), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nona22aa1n02x4               g214(.a(new_n114), .b(new_n110), .c(new_n113), .out0(new_n310));
  aoib12aa1n02x5               g215(.a(new_n113), .b(new_n112), .c(new_n111), .out0(new_n311));
  aboi22aa1n03x5               g216(.a(new_n111), .b(new_n117), .c(new_n310), .d(new_n311), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n117), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb03aa1n02x5               g218(.a(new_n101), .b(new_n117), .c(new_n102), .out0(new_n314));
  oaib12aa1n02x5               g219(.a(new_n120), .b(new_n103), .c(new_n117), .out0(new_n315));
  aboi22aa1n03x5               g220(.a(new_n99), .b(new_n100), .c(new_n118), .d(new_n119), .out0(new_n316));
  aboi22aa1n03x5               g221(.a(new_n99), .b(new_n315), .c(new_n314), .d(new_n316), .out0(\s[6] ));
  xorb03aa1n02x5               g222(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n02x5               g223(.a(new_n123), .b(new_n315), .c(new_n105), .out0(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(new_n121), .out0(\s[8] ));
  aoi112aa1n02x5               g225(.a(new_n125), .b(new_n127), .c(new_n117), .d(new_n106), .o1(new_n321));
  norb02aa1n02x5               g226(.a(new_n128), .b(new_n321), .out0(\s[9] ));
endmodule


