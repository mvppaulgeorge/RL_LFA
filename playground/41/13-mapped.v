// Benchmark "adder" written by ABC on Thu Jul 18 09:03:47 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n288, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n306, new_n308, new_n311, new_n312,
    new_n314, new_n316;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor002aa1n16x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nand02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  xnrc02aa1n06x5               g004(.a(\b[7] ), .b(\a[8] ), .out0(new_n100));
  xnrc02aa1n06x5               g005(.a(\b[6] ), .b(\a[7] ), .out0(new_n101));
  norp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  tech160nm_fixnrc02aa1n02p5x5 g007(.a(\b[5] ), .b(\a[6] ), .out0(new_n103));
  xnrc02aa1n12x5               g008(.a(\b[4] ), .b(\a[5] ), .out0(new_n104));
  nor042aa1n02x5               g009(.a(new_n104), .b(new_n103), .o1(new_n105));
  nor042aa1n04x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand02aa1d24x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  norb02aa1n12x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  nor042aa1n06x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nand02aa1n12x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  norb02aa1n06x5               g015(.a(new_n110), .b(new_n109), .out0(new_n111));
  nor022aa1n08x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  aoi022aa1d24x5               g017(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n113));
  oai112aa1n06x5               g018(.a(new_n108), .b(new_n111), .c(new_n112), .d(new_n113), .o1(new_n114));
  tech160nm_fiaoi012aa1n04x5   g019(.a(new_n106), .b(new_n109), .c(new_n107), .o1(new_n115));
  nand02aa1d04x5               g020(.a(new_n114), .b(new_n115), .o1(new_n116));
  nand23aa1n06x5               g021(.a(new_n116), .b(new_n102), .c(new_n105), .o1(new_n117));
  orn002aa1n02x5               g022(.a(\a[8] ), .b(\b[7] ), .o(new_n118));
  norp02aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  aob012aa1n02x5               g024(.a(new_n119), .b(\b[7] ), .c(\a[8] ), .out0(new_n120));
  oaih22aa1d12x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  aob012aa1n06x5               g026(.a(new_n121), .b(\b[5] ), .c(\a[6] ), .out0(new_n122));
  norp03aa1n02x5               g027(.a(new_n122), .b(new_n101), .c(new_n100), .o1(new_n123));
  nano22aa1n03x7               g028(.a(new_n123), .b(new_n118), .c(new_n120), .out0(new_n124));
  nanp02aa1n06x5               g029(.a(new_n117), .b(new_n124), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(new_n125), .b(new_n99), .o1(new_n126));
  nona22aa1n06x5               g031(.a(new_n126), .b(new_n98), .c(new_n97), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n97), .b(new_n98), .c(new_n125), .d(new_n99), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(new_n127), .b(new_n128), .o1(\s[10] ));
  nand02aa1n04x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nor002aa1d32x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand02aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xobna2aa1n03x5               g038(.a(new_n133), .b(new_n127), .c(new_n130), .out0(\s[11] ));
  nor002aa1d32x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nand02aa1d06x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanb02aa1n06x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  inv000aa1d42x5               g042(.a(new_n137), .o1(new_n138));
  aoi113aa1n02x5               g043(.a(new_n138), .b(new_n131), .c(new_n127), .d(new_n133), .e(new_n130), .o1(new_n139));
  nanp03aa1n02x5               g044(.a(new_n127), .b(new_n130), .c(new_n133), .o1(new_n140));
  oaoi13aa1n02x5               g045(.a(new_n137), .b(new_n140), .c(\a[11] ), .d(\b[10] ), .o1(new_n141));
  norp02aa1n03x5               g046(.a(new_n141), .b(new_n139), .o1(\s[12] ));
  nanb02aa1n02x5               g047(.a(new_n98), .b(new_n99), .out0(new_n143));
  nona23aa1n02x4               g048(.a(new_n133), .b(new_n138), .c(new_n97), .d(new_n143), .out0(new_n144));
  nona23aa1d18x5               g049(.a(new_n136), .b(new_n132), .c(new_n131), .d(new_n135), .out0(new_n145));
  tech160nm_fioai012aa1n03p5x5 g050(.a(new_n136), .b(new_n135), .c(new_n131), .o1(new_n146));
  norp02aa1n12x5               g051(.a(\b[9] ), .b(\a[10] ), .o1(new_n147));
  oaih12aa1n06x5               g052(.a(new_n130), .b(new_n98), .c(new_n147), .o1(new_n148));
  oai012aa1n18x5               g053(.a(new_n146), .b(new_n145), .c(new_n148), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n144), .c(new_n117), .d(new_n124), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand02aa1n06x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n151), .c(new_n154), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xorc02aa1n12x5               g061(.a(\a[15] ), .b(\b[14] ), .out0(new_n157));
  nor002aa1d32x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand42aa1d28x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nona23aa1d24x5               g064(.a(new_n159), .b(new_n154), .c(new_n153), .d(new_n158), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  oai012aa1d24x5               g066(.a(new_n159), .b(new_n158), .c(new_n153), .o1(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  aoai13aa1n06x5               g068(.a(new_n157), .b(new_n163), .c(new_n151), .d(new_n161), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n157), .b(new_n163), .c(new_n151), .d(new_n161), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(\s[15] ));
  inv000aa1d42x5               g071(.a(\a[15] ), .o1(new_n167));
  nanb02aa1n02x5               g072(.a(\b[14] ), .b(new_n167), .out0(new_n168));
  xnrc02aa1n12x5               g073(.a(\b[15] ), .b(\a[16] ), .out0(new_n169));
  nanp03aa1n02x5               g074(.a(new_n164), .b(new_n168), .c(new_n169), .o1(new_n170));
  tech160nm_fiaoi012aa1n02p5x5 g075(.a(new_n169), .b(new_n164), .c(new_n168), .o1(new_n171));
  norb02aa1n02x7               g076(.a(new_n170), .b(new_n171), .out0(\s[16] ));
  norb03aa1d15x5               g077(.a(new_n157), .b(new_n160), .c(new_n169), .out0(new_n173));
  nona32aa1n03x5               g078(.a(new_n173), .b(new_n145), .c(new_n143), .d(new_n97), .out0(new_n174));
  aoi112aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n175));
  xnrc02aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .out0(new_n176));
  orn002aa1n02x5               g081(.a(\a[16] ), .b(\b[15] ), .o(new_n177));
  oai013aa1n03x4               g082(.a(new_n177), .b(new_n176), .c(new_n169), .d(new_n162), .o1(new_n178));
  aoi112aa1n06x5               g083(.a(new_n175), .b(new_n178), .c(new_n173), .d(new_n149), .o1(new_n179));
  aoai13aa1n12x5               g084(.a(new_n179), .b(new_n174), .c(new_n117), .d(new_n124), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g086(.a(\a[17] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[16] ), .o1(new_n183));
  tech160nm_fioaoi03aa1n03p5x5 g088(.a(new_n182), .b(new_n183), .c(new_n180), .o1(new_n184));
  xnrb03aa1n03x5               g089(.a(new_n184), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  norp02aa1n02x5               g090(.a(\b[16] ), .b(\a[17] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  norp02aa1n09x5               g092(.a(\b[17] ), .b(\a[18] ), .o1(new_n188));
  nand42aa1n04x5               g093(.a(\b[17] ), .b(\a[18] ), .o1(new_n189));
  nano23aa1n06x5               g094(.a(new_n186), .b(new_n188), .c(new_n189), .d(new_n187), .out0(new_n190));
  aoai13aa1n12x5               g095(.a(new_n189), .b(new_n188), .c(new_n182), .d(new_n183), .o1(new_n191));
  inv000aa1n02x5               g096(.a(new_n191), .o1(new_n192));
  nor002aa1d24x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nand42aa1n03x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n192), .c(new_n180), .d(new_n190), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n192), .c(new_n180), .d(new_n190), .o1(new_n197));
  norb02aa1n02x7               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand42aa1n06x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nona22aa1n02x5               g107(.a(new_n196), .b(new_n202), .c(new_n193), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n193), .o1(new_n204));
  aobi12aa1n02x7               g109(.a(new_n202), .b(new_n196), .c(new_n204), .out0(new_n205));
  norb02aa1n03x4               g110(.a(new_n203), .b(new_n205), .out0(\s[20] ));
  nona23aa1n06x5               g111(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n207));
  norb02aa1n06x5               g112(.a(new_n190), .b(new_n207), .out0(new_n208));
  aoi012aa1n12x5               g113(.a(new_n200), .b(new_n193), .c(new_n201), .o1(new_n209));
  oai012aa1n09x5               g114(.a(new_n209), .b(new_n207), .c(new_n191), .o1(new_n210));
  xorc02aa1n02x5               g115(.a(\a[21] ), .b(\b[20] ), .out0(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n210), .c(new_n180), .d(new_n208), .o1(new_n212));
  aoi112aa1n02x5               g117(.a(new_n211), .b(new_n210), .c(new_n180), .d(new_n208), .o1(new_n213));
  norb02aa1n02x7               g118(.a(new_n212), .b(new_n213), .out0(\s[21] ));
  nor042aa1n03x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  xorc02aa1n02x5               g120(.a(\a[22] ), .b(\b[21] ), .out0(new_n216));
  nona22aa1n02x5               g121(.a(new_n212), .b(new_n216), .c(new_n215), .out0(new_n217));
  inv000aa1n03x5               g122(.a(new_n215), .o1(new_n218));
  aobi12aa1n02x7               g123(.a(new_n216), .b(new_n212), .c(new_n218), .out0(new_n219));
  norb02aa1n03x4               g124(.a(new_n217), .b(new_n219), .out0(\s[22] ));
  nano23aa1n03x5               g125(.a(new_n193), .b(new_n200), .c(new_n201), .d(new_n194), .out0(new_n221));
  inv000aa1d42x5               g126(.a(\a[21] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\a[22] ), .o1(new_n223));
  xroi22aa1d04x5               g128(.a(new_n222), .b(\b[20] ), .c(new_n223), .d(\b[21] ), .out0(new_n224));
  and003aa1n02x5               g129(.a(new_n224), .b(new_n221), .c(new_n190), .o(new_n225));
  oao003aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .c(new_n218), .carry(new_n226));
  inv030aa1n02x5               g131(.a(new_n226), .o1(new_n227));
  tech160nm_fiaoi012aa1n04x5   g132(.a(new_n227), .b(new_n210), .c(new_n224), .o1(new_n228));
  inv030aa1n02x5               g133(.a(new_n228), .o1(new_n229));
  xorc02aa1n12x5               g134(.a(\a[23] ), .b(\b[22] ), .out0(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n229), .c(new_n180), .d(new_n225), .o1(new_n231));
  aoi112aa1n02x5               g136(.a(new_n230), .b(new_n229), .c(new_n180), .d(new_n225), .o1(new_n232));
  norb02aa1n02x7               g137(.a(new_n231), .b(new_n232), .out0(\s[23] ));
  nor042aa1n03x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n12x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  nona22aa1n02x5               g140(.a(new_n231), .b(new_n235), .c(new_n234), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n234), .o1(new_n237));
  aobi12aa1n02x7               g142(.a(new_n235), .b(new_n231), .c(new_n237), .out0(new_n238));
  norb02aa1n03x4               g143(.a(new_n236), .b(new_n238), .out0(\s[24] ));
  and002aa1n06x5               g144(.a(new_n235), .b(new_n230), .o(new_n240));
  inv000aa1n02x5               g145(.a(new_n240), .o1(new_n241));
  nano32aa1n02x4               g146(.a(new_n241), .b(new_n224), .c(new_n221), .d(new_n190), .out0(new_n242));
  inv000aa1n02x5               g147(.a(new_n209), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n224), .b(new_n243), .c(new_n221), .d(new_n192), .o1(new_n244));
  oao003aa1n02x5               g149(.a(\a[24] ), .b(\b[23] ), .c(new_n237), .carry(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n241), .c(new_n244), .d(new_n226), .o1(new_n246));
  xorc02aa1n02x5               g151(.a(\a[25] ), .b(\b[24] ), .out0(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n246), .c(new_n180), .d(new_n242), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n247), .b(new_n246), .c(new_n180), .d(new_n242), .o1(new_n249));
  norb02aa1n02x7               g154(.a(new_n248), .b(new_n249), .out0(\s[25] ));
  nor042aa1n03x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[26] ), .b(\b[25] ), .out0(new_n252));
  nona22aa1n02x5               g157(.a(new_n248), .b(new_n252), .c(new_n251), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n251), .o1(new_n254));
  aobi12aa1n02x7               g159(.a(new_n252), .b(new_n248), .c(new_n254), .out0(new_n255));
  norb02aa1n03x4               g160(.a(new_n253), .b(new_n255), .out0(\s[26] ));
  inv030aa1n02x5               g161(.a(new_n174), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(new_n173), .b(new_n149), .o1(new_n258));
  nona22aa1n02x4               g163(.a(new_n258), .b(new_n178), .c(new_n175), .out0(new_n259));
  inv000aa1d42x5               g164(.a(\a[25] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(\a[26] ), .o1(new_n261));
  xroi22aa1d04x5               g166(.a(new_n260), .b(\b[24] ), .c(new_n261), .d(\b[25] ), .out0(new_n262));
  inv030aa1n02x5               g167(.a(new_n262), .o1(new_n263));
  nano32aa1d12x5               g168(.a(new_n263), .b(new_n208), .c(new_n224), .d(new_n240), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n259), .c(new_n125), .d(new_n257), .o1(new_n265));
  oao003aa1n03x5               g170(.a(\a[26] ), .b(\b[25] ), .c(new_n254), .carry(new_n266));
  aobi12aa1n06x5               g171(.a(new_n266), .b(new_n246), .c(new_n262), .out0(new_n267));
  xorc02aa1n12x5               g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  xnbna2aa1n03x5               g173(.a(new_n268), .b(new_n267), .c(new_n265), .out0(\s[27] ));
  nor042aa1n03x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aobi12aa1n02x7               g176(.a(new_n268), .b(new_n267), .c(new_n265), .out0(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  nano22aa1n02x4               g178(.a(new_n272), .b(new_n271), .c(new_n273), .out0(new_n274));
  aoai13aa1n03x5               g179(.a(new_n240), .b(new_n227), .c(new_n210), .d(new_n224), .o1(new_n275));
  aoai13aa1n04x5               g180(.a(new_n266), .b(new_n263), .c(new_n275), .d(new_n245), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n268), .b(new_n276), .c(new_n180), .d(new_n264), .o1(new_n277));
  aoi012aa1n03x5               g182(.a(new_n273), .b(new_n277), .c(new_n271), .o1(new_n278));
  nor002aa1n02x5               g183(.a(new_n278), .b(new_n274), .o1(\s[28] ));
  norb02aa1n02x5               g184(.a(new_n268), .b(new_n273), .out0(new_n280));
  aobi12aa1n02x7               g185(.a(new_n280), .b(new_n267), .c(new_n265), .out0(new_n281));
  oao003aa1n02x5               g186(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[28] ), .b(\a[29] ), .out0(new_n283));
  nano22aa1n02x4               g188(.a(new_n281), .b(new_n282), .c(new_n283), .out0(new_n284));
  aoai13aa1n03x5               g189(.a(new_n280), .b(new_n276), .c(new_n180), .d(new_n264), .o1(new_n285));
  aoi012aa1n03x5               g190(.a(new_n283), .b(new_n285), .c(new_n282), .o1(new_n286));
  norp02aa1n03x5               g191(.a(new_n286), .b(new_n284), .o1(\s[29] ));
  nanp02aa1n02x5               g192(.a(\b[0] ), .b(\a[1] ), .o1(new_n288));
  xorb03aa1n02x5               g193(.a(new_n288), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n268), .b(new_n283), .c(new_n273), .out0(new_n290));
  aobi12aa1n02x7               g195(.a(new_n290), .b(new_n267), .c(new_n265), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n291), .b(new_n292), .c(new_n293), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n290), .b(new_n276), .c(new_n180), .d(new_n264), .o1(new_n295));
  aoi012aa1n03x5               g200(.a(new_n293), .b(new_n295), .c(new_n292), .o1(new_n296));
  norp02aa1n03x5               g201(.a(new_n296), .b(new_n294), .o1(\s[30] ));
  norb02aa1n02x5               g202(.a(new_n290), .b(new_n293), .out0(new_n298));
  aobi12aa1n02x7               g203(.a(new_n298), .b(new_n267), .c(new_n265), .out0(new_n299));
  oao003aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .c(new_n292), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n299), .b(new_n300), .c(new_n301), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n298), .b(new_n276), .c(new_n180), .d(new_n264), .o1(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n301), .b(new_n303), .c(new_n300), .o1(new_n304));
  norp02aa1n03x5               g209(.a(new_n304), .b(new_n302), .o1(\s[31] ));
  oabi12aa1n02x5               g210(.a(new_n113), .b(\a[2] ), .c(\b[1] ), .out0(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi13aa1n02x5               g212(.a(new_n109), .b(new_n110), .c(new_n113), .d(new_n112), .o1(new_n308));
  xnrc02aa1n02x5               g213(.a(new_n308), .b(new_n108), .out0(\s[4] ));
  xobna2aa1n03x5               g214(.a(new_n104), .b(new_n114), .c(new_n115), .out0(\s[5] ));
  orn002aa1n02x5               g215(.a(\a[5] ), .b(\b[4] ), .o(new_n311));
  aoai13aa1n02x5               g216(.a(new_n311), .b(new_n104), .c(new_n114), .d(new_n115), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aob012aa1n02x5               g218(.a(new_n122), .b(new_n116), .c(new_n105), .out0(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n02x5               g220(.a(new_n119), .b(new_n314), .c(new_n101), .out0(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g222(.a(new_n143), .b(new_n117), .c(new_n124), .out0(\s[9] ));
endmodule


