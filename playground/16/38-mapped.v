// Benchmark "adder" written by ABC on Wed Jul 17 20:30:20 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n290, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n313, new_n315, new_n316, new_n318;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv030aa1d32x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d32x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  and002aa1n06x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\a[3] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[2] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  nor042aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  aoi022aa1d24x5               g012(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n108));
  nor002aa1n02x5               g013(.a(new_n108), .b(new_n107), .o1(new_n109));
  oa0022aa1n02x5               g014(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n110));
  oai012aa1n02x5               g015(.a(new_n110), .b(new_n109), .c(new_n106), .o1(new_n111));
  xnrc02aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .out0(new_n112));
  nor022aa1n04x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nand02aa1d04x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand02aa1n04x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nona23aa1d18x5               g021(.a(new_n115), .b(new_n114), .c(new_n116), .d(new_n113), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  nor043aa1n02x5               g023(.a(new_n117), .b(new_n118), .c(new_n112), .o1(new_n119));
  nanp03aa1n02x5               g024(.a(new_n119), .b(new_n101), .c(new_n111), .o1(new_n120));
  inv000aa1d42x5               g025(.a(new_n117), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\a[7] ), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\b[6] ), .o1(new_n123));
  aoai13aa1n02x5               g028(.a(new_n114), .b(new_n113), .c(new_n122), .d(new_n123), .o1(new_n124));
  orn002aa1n02x5               g029(.a(\a[5] ), .b(\b[4] ), .o(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[6] ), .b(\b[5] ), .c(new_n125), .o1(new_n126));
  aobi12aa1n02x5               g031(.a(new_n124), .b(new_n121), .c(new_n126), .out0(new_n127));
  tech160nm_fixnrc02aa1n05x5   g032(.a(\b[8] ), .b(\a[9] ), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n99), .b(new_n128), .c(new_n120), .d(new_n127), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d32x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand42aa1d28x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanb02aa1n12x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(new_n129), .b(new_n134), .o1(new_n135));
  nor042aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nanp02aa1n04x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoai13aa1n12x5               g043(.a(new_n132), .b(new_n131), .c(new_n97), .d(new_n98), .o1(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n138), .b(new_n135), .c(new_n139), .out0(\s[11] ));
  inv000aa1d42x5               g045(.a(new_n139), .o1(new_n141));
  aoai13aa1n02x5               g046(.a(new_n138), .b(new_n141), .c(new_n129), .d(new_n134), .o1(new_n142));
  nor042aa1n12x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand02aa1n04x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(new_n145));
  oai112aa1n02x5               g050(.a(new_n142), .b(new_n145), .c(\b[10] ), .d(\a[11] ), .o1(new_n146));
  oaoi13aa1n02x5               g051(.a(new_n145), .b(new_n142), .c(\a[11] ), .d(\b[10] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n146), .b(new_n147), .out0(\s[12] ));
  nano23aa1n06x5               g053(.a(new_n136), .b(new_n143), .c(new_n144), .d(new_n137), .out0(new_n149));
  nona22aa1d18x5               g054(.a(new_n149), .b(new_n128), .c(new_n133), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n143), .o1(new_n151));
  nona23aa1n09x5               g056(.a(new_n144), .b(new_n137), .c(new_n136), .d(new_n143), .out0(new_n152));
  nanp02aa1n02x5               g057(.a(new_n136), .b(new_n144), .o1(new_n153));
  oai112aa1n06x5               g058(.a(new_n153), .b(new_n151), .c(new_n152), .d(new_n139), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n150), .c(new_n120), .d(new_n127), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oaoi13aa1n02x5               g066(.a(new_n100), .b(new_n110), .c(new_n109), .d(new_n106), .o1(new_n162));
  oaib12aa1n02x7               g067(.a(new_n124), .b(new_n117), .c(new_n126), .out0(new_n163));
  inv000aa1d42x5               g068(.a(new_n150), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n162), .d(new_n119), .o1(new_n165));
  norp02aa1n02x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nona23aa1n02x4               g072(.a(new_n167), .b(new_n159), .c(new_n158), .d(new_n166), .out0(new_n168));
  oai012aa1n02x5               g073(.a(new_n167), .b(new_n166), .c(new_n158), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n169), .b(new_n168), .c(new_n165), .d(new_n155), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  norp02aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  aoi112aa1n02x5               g082(.a(new_n177), .b(new_n172), .c(new_n170), .d(new_n174), .o1(new_n178));
  aoai13aa1n02x5               g083(.a(new_n177), .b(new_n172), .c(new_n170), .d(new_n173), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(\s[16] ));
  nano23aa1n02x4               g085(.a(new_n158), .b(new_n166), .c(new_n167), .d(new_n159), .out0(new_n181));
  nano32aa1n03x7               g086(.a(new_n150), .b(new_n177), .c(new_n174), .d(new_n181), .out0(new_n182));
  aoai13aa1n09x5               g087(.a(new_n182), .b(new_n163), .c(new_n162), .d(new_n119), .o1(new_n183));
  nona23aa1n02x4               g088(.a(new_n176), .b(new_n173), .c(new_n172), .d(new_n175), .out0(new_n184));
  norp02aa1n02x5               g089(.a(new_n184), .b(new_n168), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n186));
  oai022aa1n02x5               g091(.a(new_n184), .b(new_n169), .c(\b[15] ), .d(\a[16] ), .o1(new_n187));
  aoi112aa1n09x5               g092(.a(new_n187), .b(new_n186), .c(new_n154), .d(new_n185), .o1(new_n188));
  nand02aa1d10x5               g093(.a(new_n188), .b(new_n183), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g095(.a(\a[18] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\a[17] ), .o1(new_n192));
  inv000aa1d42x5               g097(.a(\b[16] ), .o1(new_n193));
  oaoi03aa1n03x5               g098(.a(new_n192), .b(new_n193), .c(new_n189), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[17] ), .c(new_n191), .out0(\s[18] ));
  norp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nand42aa1n03x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  xroi22aa1d04x5               g103(.a(new_n192), .b(\b[16] ), .c(new_n191), .d(\b[17] ), .out0(new_n199));
  nanp02aa1n02x5               g104(.a(new_n193), .b(new_n192), .o1(new_n200));
  oaoi03aa1n02x5               g105(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n201));
  aoai13aa1n06x5               g106(.a(new_n198), .b(new_n201), .c(new_n189), .d(new_n199), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n198), .b(new_n201), .c(new_n189), .d(new_n199), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  nona22aa1n03x5               g113(.a(new_n202), .b(new_n208), .c(new_n196), .out0(new_n209));
  orn002aa1n02x5               g114(.a(\a[19] ), .b(\b[18] ), .o(new_n210));
  aobi12aa1n03x5               g115(.a(new_n208), .b(new_n202), .c(new_n210), .out0(new_n211));
  norb02aa1n03x4               g116(.a(new_n209), .b(new_n211), .out0(\s[20] ));
  nano23aa1n02x4               g117(.a(new_n196), .b(new_n206), .c(new_n207), .d(new_n197), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n199), .b(new_n213), .o1(new_n214));
  oai022aa1n02x5               g119(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n215));
  oaib12aa1n02x5               g120(.a(new_n215), .b(new_n191), .c(\b[17] ), .out0(new_n216));
  nona23aa1n03x5               g121(.a(new_n207), .b(new_n197), .c(new_n196), .d(new_n206), .out0(new_n217));
  oaoi03aa1n02x5               g122(.a(\a[20] ), .b(\b[19] ), .c(new_n210), .o1(new_n218));
  oabi12aa1n12x5               g123(.a(new_n218), .b(new_n217), .c(new_n216), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n214), .c(new_n188), .d(new_n183), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  xorc02aa1n02x5               g129(.a(\a[22] ), .b(\b[21] ), .out0(new_n225));
  aoi112aa1n02x5               g130(.a(new_n223), .b(new_n225), .c(new_n221), .d(new_n224), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n225), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n227));
  norb02aa1n03x4               g132(.a(new_n227), .b(new_n226), .out0(\s[22] ));
  inv000aa1d42x5               g133(.a(\a[21] ), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\a[22] ), .o1(new_n230));
  xroi22aa1d04x5               g135(.a(new_n229), .b(\b[20] ), .c(new_n230), .d(\b[21] ), .out0(new_n231));
  nanp03aa1n03x5               g136(.a(new_n231), .b(new_n199), .c(new_n213), .o1(new_n232));
  inv000aa1d42x5               g137(.a(\b[21] ), .o1(new_n233));
  oaoi03aa1n12x5               g138(.a(new_n230), .b(new_n233), .c(new_n223), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n219), .c(new_n231), .o1(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n232), .c(new_n188), .d(new_n183), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  xorc02aa1n02x5               g144(.a(\a[23] ), .b(\b[22] ), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  aoi112aa1n03x5               g146(.a(new_n239), .b(new_n241), .c(new_n237), .d(new_n240), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n241), .b(new_n239), .c(new_n237), .d(new_n240), .o1(new_n243));
  norb02aa1n03x4               g148(.a(new_n243), .b(new_n242), .out0(\s[24] ));
  and002aa1n12x5               g149(.a(new_n241), .b(new_n240), .o(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  nano32aa1n02x4               g151(.a(new_n246), .b(new_n231), .c(new_n199), .d(new_n213), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n231), .b(new_n218), .c(new_n213), .d(new_n201), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n249));
  oab012aa1n02x4               g154(.a(new_n249), .b(\a[24] ), .c(\b[23] ), .out0(new_n250));
  aoai13aa1n04x5               g155(.a(new_n250), .b(new_n246), .c(new_n248), .d(new_n234), .o1(new_n251));
  tech160nm_fixorc02aa1n05x5   g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n251), .c(new_n189), .d(new_n247), .o1(new_n253));
  aoi112aa1n02x5               g158(.a(new_n252), .b(new_n251), .c(new_n189), .d(new_n247), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n253), .b(new_n254), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n12x5               g161(.a(\a[26] ), .b(\b[25] ), .out0(new_n257));
  nona22aa1n02x4               g162(.a(new_n253), .b(new_n257), .c(new_n256), .out0(new_n258));
  inv000aa1n02x5               g163(.a(new_n256), .o1(new_n259));
  aobi12aa1n02x7               g164(.a(new_n257), .b(new_n253), .c(new_n259), .out0(new_n260));
  norb02aa1n03x4               g165(.a(new_n258), .b(new_n260), .out0(\s[26] ));
  and002aa1n06x5               g166(.a(new_n257), .b(new_n252), .o(new_n262));
  nano22aa1n03x7               g167(.a(new_n232), .b(new_n245), .c(new_n262), .out0(new_n263));
  nand02aa1d10x5               g168(.a(new_n189), .b(new_n263), .o1(new_n264));
  oao003aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .c(new_n259), .carry(new_n265));
  aobi12aa1n06x5               g170(.a(new_n265), .b(new_n251), .c(new_n262), .out0(new_n266));
  xorc02aa1n02x5               g171(.a(\a[27] ), .b(\b[26] ), .out0(new_n267));
  xnbna2aa1n03x5               g172(.a(new_n267), .b(new_n264), .c(new_n266), .out0(\s[27] ));
  norp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  inv040aa1n03x5               g174(.a(new_n269), .o1(new_n270));
  aobi12aa1n06x5               g175(.a(new_n267), .b(new_n264), .c(new_n266), .out0(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  nano22aa1n03x7               g177(.a(new_n271), .b(new_n270), .c(new_n272), .out0(new_n273));
  inv000aa1n02x5               g178(.a(new_n263), .o1(new_n274));
  aoi012aa1n06x5               g179(.a(new_n274), .b(new_n188), .c(new_n183), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n245), .b(new_n235), .c(new_n219), .d(new_n231), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n262), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n265), .b(new_n277), .c(new_n276), .d(new_n250), .o1(new_n278));
  oaih12aa1n02x5               g183(.a(new_n267), .b(new_n278), .c(new_n275), .o1(new_n279));
  aoi012aa1n03x5               g184(.a(new_n272), .b(new_n279), .c(new_n270), .o1(new_n280));
  norp02aa1n03x5               g185(.a(new_n280), .b(new_n273), .o1(\s[28] ));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  norb02aa1n02x5               g187(.a(new_n267), .b(new_n272), .out0(new_n283));
  oaih12aa1n02x5               g188(.a(new_n283), .b(new_n278), .c(new_n275), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n270), .carry(new_n285));
  aoi012aa1n02x5               g190(.a(new_n282), .b(new_n284), .c(new_n285), .o1(new_n286));
  aobi12aa1n06x5               g191(.a(new_n283), .b(new_n264), .c(new_n266), .out0(new_n287));
  nano22aa1n03x7               g192(.a(new_n287), .b(new_n282), .c(new_n285), .out0(new_n288));
  nor002aa1n02x5               g193(.a(new_n286), .b(new_n288), .o1(\s[29] ));
  and002aa1n02x5               g194(.a(\b[0] ), .b(\a[1] ), .o(new_n290));
  xnrb03aa1n02x5               g195(.a(new_n290), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g196(.a(new_n267), .b(new_n282), .c(new_n272), .out0(new_n292));
  oaih12aa1n02x5               g197(.a(new_n292), .b(new_n278), .c(new_n275), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n06x5               g201(.a(new_n292), .b(new_n264), .c(new_n266), .out0(new_n297));
  nano22aa1n03x7               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  norb02aa1n02x5               g205(.a(new_n292), .b(new_n295), .out0(new_n301));
  aobi12aa1n06x5               g206(.a(new_n301), .b(new_n264), .c(new_n266), .out0(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n303));
  nano22aa1n03x7               g208(.a(new_n302), .b(new_n300), .c(new_n303), .out0(new_n304));
  oaih12aa1n02x5               g209(.a(new_n301), .b(new_n278), .c(new_n275), .o1(new_n305));
  aoi012aa1n03x5               g210(.a(new_n300), .b(new_n305), .c(new_n303), .o1(new_n306));
  norp02aa1n03x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnbna2aa1n03x5               g212(.a(new_n109), .b(new_n104), .c(new_n105), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnbna2aa1n03x5               g215(.a(new_n118), .b(new_n111), .c(new_n101), .out0(\s[5] ));
  nanp02aa1n02x5               g216(.a(new_n111), .b(new_n101), .o1(new_n312));
  oaoi03aa1n02x5               g217(.a(\a[5] ), .b(\b[4] ), .c(new_n312), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g219(.a(new_n112), .b(new_n313), .out0(new_n315));
  oai012aa1n02x5               g220(.a(new_n315), .b(\b[5] ), .c(\a[6] ), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g222(.a(new_n122), .b(new_n123), .c(new_n316), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g224(.a(new_n128), .b(new_n120), .c(new_n127), .out0(\s[9] ));
endmodule


