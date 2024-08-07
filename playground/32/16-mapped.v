// Benchmark "adder" written by ABC on Thu Jul 18 04:28:44 2024

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
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n177, new_n178, new_n179, new_n180, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n209, new_n210, new_n211, new_n212, new_n213, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n224, new_n225, new_n226, new_n227, new_n228, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n242, new_n243, new_n244, new_n245, new_n246,
    new_n248, new_n249, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n271, new_n272, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n279, new_n281, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n298, new_n301, new_n302, new_n303, new_n305, new_n306,
    new_n307, new_n309;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d28x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d32x5               g002(.a(\b[8] ), .o1(new_n98));
  and002aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o(new_n99));
  inv000aa1d42x5               g004(.a(\a[3] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[2] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(new_n102), .b(new_n103), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  aoi022aa1d24x5               g010(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n106));
  nor022aa1n04x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  oa0022aa1n02x5               g012(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n108));
  oaoi13aa1n06x5               g013(.a(new_n99), .b(new_n108), .c(new_n107), .d(new_n104), .o1(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .out0(new_n110));
  norp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n03x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  xorc02aa1n02x5               g020(.a(\a[5] ), .b(\b[4] ), .out0(new_n116));
  norb03aa1n03x5               g021(.a(new_n116), .b(new_n115), .c(new_n110), .out0(new_n117));
  orn002aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .o(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  aoi012aa1n02x5               g024(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n120));
  oaib12aa1n06x5               g025(.a(new_n120), .b(new_n115), .c(new_n119), .out0(new_n121));
  tech160nm_fiao0012aa1n02p5x5 g026(.a(new_n121), .b(new_n109), .c(new_n117), .o(new_n122));
  oaoi03aa1n02x5               g027(.a(new_n97), .b(new_n98), .c(new_n122), .o1(new_n123));
  xnrb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d32x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1n20x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n126), .b(new_n125), .c(new_n97), .d(new_n98), .o1(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  norp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nano23aa1n03x7               g035(.a(new_n125), .b(new_n129), .c(new_n130), .d(new_n126), .out0(new_n131));
  aoi012aa1n02x5               g036(.a(new_n128), .b(new_n122), .c(new_n131), .o1(new_n132));
  xnrb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g038(.a(\a[11] ), .b(\b[10] ), .c(new_n132), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand42aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nor042aa1n09x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nona23aa1n09x5               g044(.a(new_n139), .b(new_n137), .c(new_n136), .d(new_n138), .out0(new_n140));
  norb02aa1n02x5               g045(.a(new_n131), .b(new_n140), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n138), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(new_n136), .b(new_n139), .o1(new_n144));
  oai112aa1n06x5               g049(.a(new_n144), .b(new_n143), .c(new_n140), .d(new_n127), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n142), .b(new_n146), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g053(.a(\a[13] ), .o1(new_n149));
  inv000aa1d42x5               g054(.a(\b[12] ), .o1(new_n150));
  oaoi03aa1n02x5               g055(.a(new_n149), .b(new_n150), .c(new_n147), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nor022aa1n08x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nona23aa1n02x4               g061(.a(new_n156), .b(new_n154), .c(new_n153), .d(new_n155), .out0(new_n157));
  aoai13aa1n02x5               g062(.a(new_n156), .b(new_n155), .c(new_n149), .d(new_n150), .o1(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n157), .c(new_n142), .d(new_n146), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  xorc02aa1n02x5               g066(.a(\a[15] ), .b(\b[14] ), .out0(new_n162));
  xorc02aa1n02x5               g067(.a(\a[16] ), .b(\b[15] ), .out0(new_n163));
  aoi112aa1n02x5               g068(.a(new_n163), .b(new_n161), .c(new_n159), .d(new_n162), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n163), .b(new_n161), .c(new_n159), .d(new_n162), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(\s[16] ));
  nano23aa1n02x4               g071(.a(new_n136), .b(new_n138), .c(new_n139), .d(new_n137), .out0(new_n167));
  nanp02aa1n02x5               g072(.a(new_n163), .b(new_n162), .o1(new_n168));
  nano23aa1n03x7               g073(.a(new_n168), .b(new_n157), .c(new_n167), .d(new_n131), .out0(new_n169));
  aoai13aa1n09x5               g074(.a(new_n169), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n170));
  nano22aa1n02x4               g075(.a(new_n157), .b(new_n162), .c(new_n163), .out0(new_n171));
  aoi112aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n172));
  oai022aa1n02x5               g077(.a(new_n168), .b(new_n158), .c(\b[15] ), .d(\a[16] ), .o1(new_n173));
  aoi112aa1n09x5               g078(.a(new_n173), .b(new_n172), .c(new_n145), .d(new_n171), .o1(new_n174));
  nand02aa1d10x5               g079(.a(new_n174), .b(new_n170), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g081(.a(\a[18] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\a[17] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\b[16] ), .o1(new_n179));
  oaoi03aa1n03x5               g084(.a(new_n178), .b(new_n179), .c(new_n175), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[17] ), .c(new_n177), .out0(\s[18] ));
  xroi22aa1d04x5               g086(.a(new_n178), .b(\b[16] ), .c(new_n177), .d(\b[17] ), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n179), .b(new_n178), .o1(new_n183));
  oaoi03aa1n02x5               g088(.a(\a[18] ), .b(\b[17] ), .c(new_n183), .o1(new_n184));
  nor022aa1n03x5               g089(.a(\b[18] ), .b(\a[19] ), .o1(new_n185));
  nand42aa1n06x5               g090(.a(\b[18] ), .b(\a[19] ), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n186), .b(new_n185), .out0(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n184), .c(new_n175), .d(new_n182), .o1(new_n188));
  aoi112aa1n02x5               g093(.a(new_n187), .b(new_n184), .c(new_n175), .d(new_n182), .o1(new_n189));
  norb02aa1n02x5               g094(.a(new_n188), .b(new_n189), .out0(\s[19] ));
  xnrc02aa1n02x5               g095(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n02x5               g096(.a(\b[19] ), .b(\a[20] ), .o1(new_n192));
  nand42aa1n03x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  nona22aa1n03x5               g099(.a(new_n188), .b(new_n194), .c(new_n185), .out0(new_n195));
  orn002aa1n02x5               g100(.a(\a[19] ), .b(\b[18] ), .o(new_n196));
  aobi12aa1n03x5               g101(.a(new_n194), .b(new_n188), .c(new_n196), .out0(new_n197));
  norb02aa1n03x4               g102(.a(new_n195), .b(new_n197), .out0(\s[20] ));
  nano23aa1n03x5               g103(.a(new_n185), .b(new_n192), .c(new_n193), .d(new_n186), .out0(new_n199));
  nanp02aa1n02x5               g104(.a(new_n182), .b(new_n199), .o1(new_n200));
  oai022aa1n02x5               g105(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n201));
  oaib12aa1n02x5               g106(.a(new_n201), .b(new_n177), .c(\b[17] ), .out0(new_n202));
  nona23aa1n03x5               g107(.a(new_n193), .b(new_n186), .c(new_n185), .d(new_n192), .out0(new_n203));
  oaoi03aa1n02x5               g108(.a(\a[20] ), .b(\b[19] ), .c(new_n196), .o1(new_n204));
  oabi12aa1n12x5               g109(.a(new_n204), .b(new_n203), .c(new_n202), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  aoai13aa1n03x5               g111(.a(new_n206), .b(new_n200), .c(new_n174), .d(new_n170), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g113(.a(\b[20] ), .b(\a[21] ), .o1(new_n209));
  xorc02aa1n02x5               g114(.a(\a[21] ), .b(\b[20] ), .out0(new_n210));
  xorc02aa1n02x5               g115(.a(\a[22] ), .b(\b[21] ), .out0(new_n211));
  aoi112aa1n02x7               g116(.a(new_n209), .b(new_n211), .c(new_n207), .d(new_n210), .o1(new_n212));
  aoai13aa1n03x5               g117(.a(new_n211), .b(new_n209), .c(new_n207), .d(new_n210), .o1(new_n213));
  norb02aa1n03x4               g118(.a(new_n213), .b(new_n212), .out0(\s[22] ));
  inv000aa1d42x5               g119(.a(\a[21] ), .o1(new_n215));
  inv000aa1d42x5               g120(.a(\a[22] ), .o1(new_n216));
  xroi22aa1d04x5               g121(.a(new_n215), .b(\b[20] ), .c(new_n216), .d(\b[21] ), .out0(new_n217));
  inv000aa1d42x5               g122(.a(\b[21] ), .o1(new_n218));
  oao003aa1n02x5               g123(.a(new_n216), .b(new_n218), .c(new_n209), .carry(new_n219));
  aoi012aa1n02x5               g124(.a(new_n219), .b(new_n205), .c(new_n217), .o1(new_n220));
  nanp03aa1n02x5               g125(.a(new_n217), .b(new_n182), .c(new_n199), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n220), .b(new_n221), .c(new_n174), .d(new_n170), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g128(.a(\b[22] ), .b(\a[23] ), .o1(new_n224));
  xorc02aa1n03x5               g129(.a(\a[23] ), .b(\b[22] ), .out0(new_n225));
  xorc02aa1n03x5               g130(.a(\a[24] ), .b(\b[23] ), .out0(new_n226));
  aoi112aa1n02x7               g131(.a(new_n224), .b(new_n226), .c(new_n222), .d(new_n225), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n226), .b(new_n224), .c(new_n222), .d(new_n225), .o1(new_n228));
  norb02aa1n03x4               g133(.a(new_n228), .b(new_n227), .out0(\s[24] ));
  and002aa1n02x7               g134(.a(new_n226), .b(new_n225), .o(new_n230));
  inv000aa1n02x5               g135(.a(new_n230), .o1(new_n231));
  nano32aa1n02x4               g136(.a(new_n231), .b(new_n217), .c(new_n182), .d(new_n199), .out0(new_n232));
  aoai13aa1n03x5               g137(.a(new_n217), .b(new_n204), .c(new_n199), .d(new_n184), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n219), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n235));
  oab012aa1n02x4               g140(.a(new_n235), .b(\a[24] ), .c(\b[23] ), .out0(new_n236));
  aoai13aa1n03x5               g141(.a(new_n236), .b(new_n231), .c(new_n233), .d(new_n234), .o1(new_n237));
  xorc02aa1n02x5               g142(.a(\a[25] ), .b(\b[24] ), .out0(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n237), .c(new_n175), .d(new_n232), .o1(new_n239));
  aoi112aa1n02x5               g144(.a(new_n238), .b(new_n237), .c(new_n175), .d(new_n232), .o1(new_n240));
  norb02aa1n02x5               g145(.a(new_n239), .b(new_n240), .out0(\s[25] ));
  nor042aa1n03x5               g146(.a(\b[24] ), .b(\a[25] ), .o1(new_n242));
  xorc02aa1n02x5               g147(.a(\a[26] ), .b(\b[25] ), .out0(new_n243));
  nona22aa1n03x5               g148(.a(new_n239), .b(new_n243), .c(new_n242), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n242), .o1(new_n245));
  aobi12aa1n03x5               g150(.a(new_n243), .b(new_n239), .c(new_n245), .out0(new_n246));
  norb02aa1n03x4               g151(.a(new_n244), .b(new_n246), .out0(\s[26] ));
  inv000aa1d42x5               g152(.a(\a[25] ), .o1(new_n248));
  inv000aa1d42x5               g153(.a(\a[26] ), .o1(new_n249));
  xroi22aa1d06x4               g154(.a(new_n248), .b(\b[24] ), .c(new_n249), .d(\b[25] ), .out0(new_n250));
  nano22aa1n03x7               g155(.a(new_n221), .b(new_n230), .c(new_n250), .out0(new_n251));
  nand02aa1d10x5               g156(.a(new_n175), .b(new_n251), .o1(new_n252));
  oao003aa1n02x5               g157(.a(\a[26] ), .b(\b[25] ), .c(new_n245), .carry(new_n253));
  aobi12aa1n06x5               g158(.a(new_n253), .b(new_n237), .c(new_n250), .out0(new_n254));
  norp02aa1n02x5               g159(.a(\b[26] ), .b(\a[27] ), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(\b[26] ), .b(\a[27] ), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(new_n257));
  xnbna2aa1n03x5               g162(.a(new_n257), .b(new_n252), .c(new_n254), .out0(\s[27] ));
  inv000aa1n06x5               g163(.a(new_n255), .o1(new_n259));
  xnrc02aa1n02x5               g164(.a(\b[27] ), .b(\a[28] ), .out0(new_n260));
  inv000aa1n02x5               g165(.a(new_n251), .o1(new_n261));
  tech160nm_fiaoi012aa1n05x5   g166(.a(new_n261), .b(new_n174), .c(new_n170), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n230), .b(new_n219), .c(new_n205), .d(new_n217), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n250), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n253), .b(new_n264), .c(new_n263), .d(new_n236), .o1(new_n265));
  oaih12aa1n02x5               g170(.a(new_n256), .b(new_n265), .c(new_n262), .o1(new_n266));
  aoi012aa1n03x5               g171(.a(new_n260), .b(new_n266), .c(new_n259), .o1(new_n267));
  aoi022aa1n06x5               g172(.a(new_n252), .b(new_n254), .c(\b[26] ), .d(\a[27] ), .o1(new_n268));
  nano22aa1n03x7               g173(.a(new_n268), .b(new_n259), .c(new_n260), .out0(new_n269));
  nor002aa1n02x5               g174(.a(new_n267), .b(new_n269), .o1(\s[28] ));
  nano22aa1n02x4               g175(.a(new_n260), .b(new_n259), .c(new_n256), .out0(new_n271));
  oaih12aa1n02x5               g176(.a(new_n271), .b(new_n265), .c(new_n262), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[28] ), .b(\b[27] ), .c(new_n259), .carry(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[28] ), .b(\a[29] ), .out0(new_n274));
  aoi012aa1n03x5               g179(.a(new_n274), .b(new_n272), .c(new_n273), .o1(new_n275));
  aobi12aa1n06x5               g180(.a(new_n271), .b(new_n252), .c(new_n254), .out0(new_n276));
  nano22aa1n03x7               g181(.a(new_n276), .b(new_n273), .c(new_n274), .out0(new_n277));
  norp02aa1n03x5               g182(.a(new_n275), .b(new_n277), .o1(\s[29] ));
  and002aa1n02x5               g183(.a(\b[0] ), .b(\a[1] ), .o(new_n279));
  xnrb03aa1n02x5               g184(.a(new_n279), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g185(.a(new_n257), .b(new_n274), .c(new_n260), .out0(new_n281));
  oaih12aa1n02x5               g186(.a(new_n281), .b(new_n265), .c(new_n262), .o1(new_n282));
  oao003aa1n02x5               g187(.a(\a[29] ), .b(\b[28] ), .c(new_n273), .carry(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[29] ), .b(\a[30] ), .out0(new_n284));
  aoi012aa1n03x5               g189(.a(new_n284), .b(new_n282), .c(new_n283), .o1(new_n285));
  aobi12aa1n06x5               g190(.a(new_n281), .b(new_n252), .c(new_n254), .out0(new_n286));
  nano22aa1n03x7               g191(.a(new_n286), .b(new_n283), .c(new_n284), .out0(new_n287));
  norp02aa1n03x5               g192(.a(new_n285), .b(new_n287), .o1(\s[30] ));
  xnrc02aa1n02x5               g193(.a(\b[30] ), .b(\a[31] ), .out0(new_n289));
  norb03aa1n02x5               g194(.a(new_n271), .b(new_n284), .c(new_n274), .out0(new_n290));
  aobi12aa1n06x5               g195(.a(new_n290), .b(new_n252), .c(new_n254), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[30] ), .b(\b[29] ), .c(new_n283), .carry(new_n292));
  nano22aa1n03x7               g197(.a(new_n291), .b(new_n289), .c(new_n292), .out0(new_n293));
  oaih12aa1n02x5               g198(.a(new_n290), .b(new_n265), .c(new_n262), .o1(new_n294));
  aoi012aa1n03x5               g199(.a(new_n289), .b(new_n294), .c(new_n292), .o1(new_n295));
  norp02aa1n03x5               g200(.a(new_n295), .b(new_n293), .o1(\s[31] ));
  xnbna2aa1n03x5               g201(.a(new_n107), .b(new_n102), .c(new_n103), .out0(\s[3] ));
  oaoi03aa1n02x5               g202(.a(\a[3] ), .b(\b[2] ), .c(new_n107), .o1(new_n298));
  xorb03aa1n02x5               g203(.a(new_n298), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g204(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g205(.a(new_n108), .b(new_n107), .c(new_n104), .o1(new_n301));
  nanb02aa1n02x5               g206(.a(new_n99), .b(new_n301), .out0(new_n302));
  oaoi03aa1n02x5               g207(.a(\a[5] ), .b(\b[4] ), .c(new_n302), .o1(new_n303));
  xorb03aa1n02x5               g208(.a(new_n303), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g209(.a(new_n113), .o1(new_n305));
  nanp02aa1n02x5               g210(.a(\b[5] ), .b(\a[6] ), .o1(new_n306));
  oai012aa1n02x5               g211(.a(new_n306), .b(new_n303), .c(new_n110), .o1(new_n307));
  xnbna2aa1n03x5               g212(.a(new_n307), .b(new_n305), .c(new_n114), .out0(\s[7] ));
  oaoi03aa1n02x5               g213(.a(\a[7] ), .b(\b[6] ), .c(new_n307), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g215(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


