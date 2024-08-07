// Benchmark "adder" written by ABC on Wed Jul 17 14:44:49 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n328, new_n331, new_n333, new_n334,
    new_n336;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d30x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1n18x5               g002(.a(\b[8] ), .o1(new_n98));
  and002aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o(new_n99));
  oaoi03aa1n09x5               g004(.a(\a[2] ), .b(\b[1] ), .c(new_n99), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand02aa1n04x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norb02aa1n06x5               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  nor042aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand22aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n09x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nand23aa1n06x5               g011(.a(new_n100), .b(new_n103), .c(new_n106), .o1(new_n107));
  aoi012aa1n06x5               g012(.a(new_n101), .b(new_n104), .c(new_n102), .o1(new_n108));
  nand42aa1d28x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor042aa1n04x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor042aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand42aa1n08x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1n09x5               g017(.a(new_n111), .b(new_n110), .c(new_n112), .d(new_n109), .out0(new_n113));
  tech160nm_fixorc02aa1n05x5   g018(.a(\a[6] ), .b(\b[5] ), .out0(new_n114));
  xorc02aa1n12x5               g019(.a(\a[5] ), .b(\b[4] ), .out0(new_n115));
  nand23aa1n02x5               g020(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n116));
  nor042aa1d18x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  inv030aa1n08x5               g022(.a(new_n117), .o1(new_n118));
  oaoi03aa1n03x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  oai022aa1n02x5               g024(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n120));
  aoi022aa1n06x5               g025(.a(new_n113), .b(new_n119), .c(new_n109), .d(new_n120), .o1(new_n121));
  aoai13aa1n12x5               g026(.a(new_n121), .b(new_n116), .c(new_n107), .d(new_n108), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(new_n97), .b(new_n98), .c(new_n122), .o1(new_n123));
  xnrb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n24x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1n06x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanb02aa1n03x5               g031(.a(new_n125), .b(new_n126), .out0(new_n127));
  tech160nm_fixnrc02aa1n02p5x5 g032(.a(\b[8] ), .b(\a[9] ), .out0(new_n128));
  nona22aa1n03x5               g033(.a(new_n122), .b(new_n128), .c(new_n127), .out0(new_n129));
  aoai13aa1n06x5               g034(.a(new_n126), .b(new_n125), .c(new_n97), .d(new_n98), .o1(new_n130));
  nor002aa1d32x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand02aa1d16x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n129), .c(new_n130), .out0(\s[11] ));
  aob012aa1n03x5               g040(.a(new_n134), .b(new_n129), .c(new_n130), .out0(new_n136));
  nor002aa1d32x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand02aa1d28x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(new_n139));
  oaib12aa1n06x5               g044(.a(new_n139), .b(new_n131), .c(new_n136), .out0(new_n140));
  nona22aa1n02x4               g045(.a(new_n136), .b(new_n139), .c(new_n131), .out0(new_n141));
  nanp02aa1n03x5               g046(.a(new_n140), .b(new_n141), .o1(\s[12] ));
  nano23aa1n09x5               g047(.a(new_n131), .b(new_n137), .c(new_n138), .d(new_n132), .out0(new_n143));
  nona22aa1n02x4               g048(.a(new_n143), .b(new_n128), .c(new_n127), .out0(new_n144));
  nanb02aa1n06x5               g049(.a(new_n144), .b(new_n122), .out0(new_n145));
  nona23aa1d18x5               g050(.a(new_n138), .b(new_n132), .c(new_n131), .d(new_n137), .out0(new_n146));
  ao0012aa1n12x5               g051(.a(new_n137), .b(new_n131), .c(new_n138), .o(new_n147));
  oabi12aa1n18x5               g052(.a(new_n147), .b(new_n146), .c(new_n130), .out0(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  nor042aa1n06x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nand42aa1d28x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nanb02aa1n02x5               g056(.a(new_n150), .b(new_n151), .out0(new_n152));
  xobna2aa1n03x5               g057(.a(new_n152), .b(new_n145), .c(new_n149), .out0(\s[13] ));
  nand22aa1n03x5               g058(.a(new_n145), .b(new_n149), .o1(new_n154));
  tech160nm_fiaoi012aa1n05x5   g059(.a(new_n150), .b(new_n154), .c(new_n151), .o1(new_n155));
  xnrb03aa1n03x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nand42aa1n16x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nano23aa1d15x5               g063(.a(new_n150), .b(new_n157), .c(new_n158), .d(new_n151), .out0(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  oai012aa1n04x7               g065(.a(new_n158), .b(new_n157), .c(new_n150), .o1(new_n161));
  aoai13aa1n06x5               g066(.a(new_n161), .b(new_n160), .c(new_n145), .d(new_n149), .o1(new_n162));
  nor002aa1n12x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nand42aa1d28x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  inv000aa1n02x5               g071(.a(new_n161), .o1(new_n167));
  aoi112aa1n02x5               g072(.a(new_n166), .b(new_n167), .c(new_n154), .d(new_n159), .o1(new_n168));
  aoi012aa1n02x5               g073(.a(new_n168), .b(new_n162), .c(new_n166), .o1(\s[15] ));
  nor042aa1n06x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1d28x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  aoai13aa1n03x5               g077(.a(new_n172), .b(new_n163), .c(new_n162), .d(new_n166), .o1(new_n173));
  tech160nm_finand02aa1n03p5x5 g078(.a(new_n162), .b(new_n166), .o1(new_n174));
  nona22aa1n03x5               g079(.a(new_n174), .b(new_n172), .c(new_n163), .out0(new_n175));
  nanp02aa1n03x5               g080(.a(new_n175), .b(new_n173), .o1(\s[16] ));
  inv040aa1d32x5               g081(.a(\a[17] ), .o1(new_n177));
  nano23aa1d15x5               g082(.a(new_n163), .b(new_n170), .c(new_n171), .d(new_n164), .out0(new_n178));
  nano22aa1n12x5               g083(.a(new_n144), .b(new_n159), .c(new_n178), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n178), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n98), .b(new_n97), .o1(new_n181));
  oaoi03aa1n02x5               g086(.a(\a[10] ), .b(\b[9] ), .c(new_n181), .o1(new_n182));
  aoai13aa1n06x5               g087(.a(new_n159), .b(new_n147), .c(new_n143), .d(new_n182), .o1(new_n183));
  tech160nm_fiaoi012aa1n03p5x5 g088(.a(new_n180), .b(new_n183), .c(new_n161), .o1(new_n184));
  aoi012aa1d24x5               g089(.a(new_n170), .b(new_n163), .c(new_n171), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n185), .o1(new_n186));
  aoi112aa1n09x5               g091(.a(new_n184), .b(new_n186), .c(new_n122), .d(new_n179), .o1(new_n187));
  xorb03aa1n03x5               g092(.a(new_n187), .b(\b[16] ), .c(new_n177), .out0(\s[17] ));
  inv040aa1n12x5               g093(.a(\b[16] ), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(new_n189), .b(new_n177), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n185), .b(new_n180), .c(new_n183), .d(new_n161), .o1(new_n191));
  xorc02aa1n12x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  aoai13aa1n03x5               g097(.a(new_n192), .b(new_n191), .c(new_n122), .d(new_n179), .o1(new_n193));
  nor002aa1d32x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  nand42aa1n10x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nanb02aa1d24x5               g100(.a(new_n194), .b(new_n195), .out0(new_n196));
  xobna2aa1n03x5               g101(.a(new_n196), .b(new_n193), .c(new_n190), .out0(\s[18] ));
  inv000aa1d42x5               g102(.a(\a[18] ), .o1(new_n198));
  xroi22aa1d04x5               g103(.a(new_n177), .b(\b[16] ), .c(new_n198), .d(\b[17] ), .out0(new_n199));
  inv030aa1n02x5               g104(.a(new_n199), .o1(new_n200));
  aoai13aa1n06x5               g105(.a(new_n195), .b(new_n194), .c(new_n177), .d(new_n189), .o1(new_n201));
  tech160nm_fioai012aa1n04x5   g106(.a(new_n201), .b(new_n187), .c(new_n200), .o1(new_n202));
  nor042aa1d18x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand02aa1d16x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  norb02aa1n06x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  nanp02aa1n06x5               g110(.a(new_n122), .b(new_n179), .o1(new_n206));
  aoai13aa1n09x5               g111(.a(new_n178), .b(new_n167), .c(new_n148), .d(new_n159), .o1(new_n207));
  nand23aa1n06x5               g112(.a(new_n206), .b(new_n207), .c(new_n185), .o1(new_n208));
  oaoi03aa1n12x5               g113(.a(\a[18] ), .b(\b[17] ), .c(new_n190), .o1(new_n209));
  aoi112aa1n03x4               g114(.a(new_n205), .b(new_n209), .c(new_n208), .d(new_n199), .o1(new_n210));
  tech160nm_fiaoi012aa1n02p5x5 g115(.a(new_n210), .b(new_n202), .c(new_n205), .o1(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand02aa1d28x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanb02aa1n12x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n203), .c(new_n202), .d(new_n205), .o1(new_n216));
  aoai13aa1n02x7               g121(.a(new_n205), .b(new_n209), .c(new_n208), .d(new_n199), .o1(new_n217));
  nona22aa1n03x5               g122(.a(new_n217), .b(new_n215), .c(new_n203), .out0(new_n218));
  nanp02aa1n03x5               g123(.a(new_n216), .b(new_n218), .o1(\s[20] ));
  nona23aa1d18x5               g124(.a(new_n205), .b(new_n192), .c(new_n215), .d(new_n196), .out0(new_n220));
  nona23aa1n09x5               g125(.a(new_n214), .b(new_n204), .c(new_n203), .d(new_n213), .out0(new_n221));
  ao0012aa1n03x7               g126(.a(new_n213), .b(new_n203), .c(new_n214), .o(new_n222));
  oabi12aa1n18x5               g127(.a(new_n222), .b(new_n221), .c(new_n201), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  tech160nm_fioai012aa1n05x5   g129(.a(new_n224), .b(new_n187), .c(new_n220), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[20] ), .b(\a[21] ), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n220), .o1(new_n228));
  aoi112aa1n03x4               g133(.a(new_n227), .b(new_n223), .c(new_n208), .d(new_n228), .o1(new_n229));
  tech160nm_fiaoi012aa1n02p5x5 g134(.a(new_n229), .b(new_n225), .c(new_n227), .o1(\s[21] ));
  norp02aa1n04x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[21] ), .b(\a[22] ), .out0(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n231), .c(new_n225), .d(new_n227), .o1(new_n233));
  aoai13aa1n04x5               g138(.a(new_n227), .b(new_n223), .c(new_n208), .d(new_n228), .o1(new_n234));
  nona22aa1n03x5               g139(.a(new_n234), .b(new_n232), .c(new_n231), .out0(new_n235));
  nanp02aa1n03x5               g140(.a(new_n233), .b(new_n235), .o1(\s[22] ));
  nano23aa1n03x7               g141(.a(new_n203), .b(new_n213), .c(new_n214), .d(new_n204), .out0(new_n237));
  nor042aa1n06x5               g142(.a(new_n232), .b(new_n226), .o1(new_n238));
  nano22aa1n03x7               g143(.a(new_n200), .b(new_n238), .c(new_n237), .out0(new_n239));
  inv000aa1n02x5               g144(.a(new_n239), .o1(new_n240));
  inv000aa1d42x5               g145(.a(\a[22] ), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\b[21] ), .o1(new_n242));
  oaoi03aa1n09x5               g147(.a(new_n241), .b(new_n242), .c(new_n231), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  aoi012aa1n09x5               g149(.a(new_n244), .b(new_n223), .c(new_n238), .o1(new_n245));
  tech160nm_fioai012aa1n05x5   g150(.a(new_n245), .b(new_n187), .c(new_n240), .o1(new_n246));
  xorc02aa1n12x5               g151(.a(\a[23] ), .b(\b[22] ), .out0(new_n247));
  inv000aa1n02x5               g152(.a(new_n245), .o1(new_n248));
  aoi112aa1n03x4               g153(.a(new_n247), .b(new_n248), .c(new_n208), .d(new_n239), .o1(new_n249));
  tech160nm_fiaoi012aa1n02p5x5 g154(.a(new_n249), .b(new_n246), .c(new_n247), .o1(\s[23] ));
  norp02aa1n02x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  xnrc02aa1n12x5               g156(.a(\b[23] ), .b(\a[24] ), .out0(new_n252));
  aoai13aa1n04x5               g157(.a(new_n252), .b(new_n251), .c(new_n246), .d(new_n247), .o1(new_n253));
  aoai13aa1n02x7               g158(.a(new_n247), .b(new_n248), .c(new_n208), .d(new_n239), .o1(new_n254));
  nona22aa1n03x5               g159(.a(new_n254), .b(new_n252), .c(new_n251), .out0(new_n255));
  nanp02aa1n03x5               g160(.a(new_n253), .b(new_n255), .o1(\s[24] ));
  norb02aa1n03x4               g161(.a(new_n247), .b(new_n252), .out0(new_n257));
  inv030aa1n04x5               g162(.a(new_n257), .o1(new_n258));
  nano32aa1n03x7               g163(.a(new_n258), .b(new_n199), .c(new_n238), .d(new_n237), .out0(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  aoai13aa1n06x5               g165(.a(new_n238), .b(new_n222), .c(new_n237), .d(new_n209), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n262));
  oab012aa1n02x4               g167(.a(new_n262), .b(\a[24] ), .c(\b[23] ), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n258), .c(new_n261), .d(new_n243), .o1(new_n264));
  inv000aa1n02x5               g169(.a(new_n264), .o1(new_n265));
  tech160nm_fioai012aa1n05x5   g170(.a(new_n265), .b(new_n187), .c(new_n260), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor002aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xnrc02aa1n12x5               g174(.a(\b[25] ), .b(\a[26] ), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n270), .b(new_n268), .c(new_n266), .d(new_n269), .o1(new_n271));
  aoai13aa1n02x7               g176(.a(new_n269), .b(new_n264), .c(new_n208), .d(new_n259), .o1(new_n272));
  nona22aa1n03x5               g177(.a(new_n272), .b(new_n270), .c(new_n268), .out0(new_n273));
  nanp02aa1n03x5               g178(.a(new_n271), .b(new_n273), .o1(\s[26] ));
  norb02aa1n12x5               g179(.a(new_n269), .b(new_n270), .out0(new_n275));
  nano23aa1d15x5               g180(.a(new_n220), .b(new_n258), .c(new_n275), .d(new_n238), .out0(new_n276));
  aoai13aa1n03x5               g181(.a(new_n276), .b(new_n191), .c(new_n122), .d(new_n179), .o1(new_n277));
  inv000aa1n02x5               g182(.a(new_n276), .o1(new_n278));
  inv040aa1d32x5               g183(.a(\a[26] ), .o1(new_n279));
  inv000aa1d42x5               g184(.a(\b[25] ), .o1(new_n280));
  oaoi03aa1n03x5               g185(.a(new_n279), .b(new_n280), .c(new_n268), .o1(new_n281));
  inv000aa1n02x5               g186(.a(new_n281), .o1(new_n282));
  aoi012aa1n06x5               g187(.a(new_n282), .b(new_n264), .c(new_n275), .o1(new_n283));
  tech160nm_fioai012aa1n05x5   g188(.a(new_n283), .b(new_n187), .c(new_n278), .o1(new_n284));
  xorc02aa1n12x5               g189(.a(\a[27] ), .b(\b[26] ), .out0(new_n285));
  aoi112aa1n02x7               g190(.a(new_n285), .b(new_n282), .c(new_n264), .d(new_n275), .o1(new_n286));
  aoi022aa1n03x5               g191(.a(new_n284), .b(new_n285), .c(new_n277), .d(new_n286), .o1(\s[27] ));
  norp02aa1n02x5               g192(.a(\b[26] ), .b(\a[27] ), .o1(new_n288));
  norp02aa1n02x5               g193(.a(\b[27] ), .b(\a[28] ), .o1(new_n289));
  nand42aa1n03x5               g194(.a(\b[27] ), .b(\a[28] ), .o1(new_n290));
  nanb02aa1n06x5               g195(.a(new_n289), .b(new_n290), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n291), .b(new_n288), .c(new_n284), .d(new_n285), .o1(new_n292));
  aoai13aa1n06x5               g197(.a(new_n257), .b(new_n244), .c(new_n223), .d(new_n238), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n275), .o1(new_n294));
  aoai13aa1n04x5               g199(.a(new_n281), .b(new_n294), .c(new_n293), .d(new_n263), .o1(new_n295));
  aoai13aa1n04x5               g200(.a(new_n285), .b(new_n295), .c(new_n208), .d(new_n276), .o1(new_n296));
  nona22aa1n02x4               g201(.a(new_n296), .b(new_n291), .c(new_n288), .out0(new_n297));
  nanp02aa1n03x5               g202(.a(new_n292), .b(new_n297), .o1(\s[28] ));
  norb02aa1n03x5               g203(.a(new_n285), .b(new_n291), .out0(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n295), .c(new_n208), .d(new_n276), .o1(new_n300));
  nor002aa1n02x5               g205(.a(\b[28] ), .b(\a[29] ), .o1(new_n301));
  nand42aa1n06x5               g206(.a(\b[28] ), .b(\a[29] ), .o1(new_n302));
  norb02aa1n02x5               g207(.a(new_n302), .b(new_n301), .out0(new_n303));
  oai022aa1n02x5               g208(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n304));
  aboi22aa1n03x5               g209(.a(new_n301), .b(new_n302), .c(new_n304), .d(new_n290), .out0(new_n305));
  inv000aa1d42x5               g210(.a(new_n299), .o1(new_n306));
  oai012aa1n02x5               g211(.a(new_n290), .b(new_n289), .c(new_n288), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n306), .c(new_n277), .d(new_n283), .o1(new_n308));
  aoi022aa1n03x5               g213(.a(new_n308), .b(new_n303), .c(new_n300), .d(new_n305), .o1(\s[29] ));
  xnrb03aa1n02x5               g214(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g215(.a(new_n291), .b(new_n285), .c(new_n303), .out0(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n295), .c(new_n208), .d(new_n276), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .out0(new_n313));
  aoi113aa1n02x5               g218(.a(new_n313), .b(new_n301), .c(new_n302), .d(new_n304), .e(new_n290), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n311), .o1(new_n315));
  aoi013aa1n02x4               g220(.a(new_n301), .b(new_n304), .c(new_n302), .d(new_n290), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n315), .c(new_n277), .d(new_n283), .o1(new_n317));
  aoi022aa1n03x5               g222(.a(new_n317), .b(new_n313), .c(new_n312), .d(new_n314), .o1(\s[30] ));
  nanp03aa1n02x5               g223(.a(new_n299), .b(new_n303), .c(new_n313), .o1(new_n319));
  nanb02aa1n02x5               g224(.a(new_n319), .b(new_n284), .out0(new_n320));
  xorc02aa1n02x5               g225(.a(\a[31] ), .b(\b[30] ), .out0(new_n321));
  oao003aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .c(new_n316), .carry(new_n322));
  norb02aa1n02x5               g227(.a(new_n322), .b(new_n321), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n322), .b(new_n319), .c(new_n277), .d(new_n283), .o1(new_n324));
  aoi022aa1n03x5               g229(.a(new_n324), .b(new_n321), .c(new_n320), .d(new_n323), .o1(\s[31] ));
  xorb03aa1n02x5               g230(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanp02aa1n02x5               g231(.a(new_n107), .b(new_n108), .o1(new_n327));
  aoi112aa1n02x5               g232(.a(new_n104), .b(new_n103), .c(new_n100), .d(new_n105), .o1(new_n328));
  oaoi13aa1n02x5               g233(.a(new_n328), .b(new_n327), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g234(.a(new_n115), .b(new_n107), .c(new_n108), .out0(\s[5] ));
  nanp02aa1n02x5               g235(.a(new_n327), .b(new_n115), .o1(new_n331));
  xnbna2aa1n03x5               g236(.a(new_n114), .b(new_n331), .c(new_n118), .out0(\s[6] ));
  norb02aa1n02x5               g237(.a(new_n114), .b(new_n117), .out0(new_n333));
  ao0022aa1n03x5               g238(.a(new_n331), .b(new_n333), .c(\b[5] ), .d(\a[6] ), .o(new_n334));
  xnrb03aa1n02x5               g239(.a(new_n334), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g240(.a(\a[7] ), .b(\b[6] ), .c(new_n334), .o1(new_n336));
  xorb03aa1n02x5               g241(.a(new_n336), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g242(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


