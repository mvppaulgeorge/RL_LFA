// Benchmark "adder" written by ABC on Thu Jul 18 09:01:20 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n288, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n305, new_n307, new_n310, new_n311, new_n313,
    new_n315;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1d18x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  inv030aa1n03x5               g004(.a(new_n99), .o1(new_n100));
  xorc02aa1n12x5               g005(.a(\a[8] ), .b(\b[7] ), .out0(new_n101));
  xorc02aa1n12x5               g006(.a(\a[7] ), .b(\b[6] ), .out0(new_n102));
  nand42aa1n16x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  aoi112aa1n09x5               g008(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n104));
  oab012aa1n06x5               g009(.a(new_n104), .b(\a[6] ), .c(\b[5] ), .out0(new_n105));
  inv030aa1n04x5               g010(.a(\a[8] ), .o1(new_n106));
  nor042aa1n04x5               g011(.a(\b[6] ), .b(\a[7] ), .o1(new_n107));
  aob012aa1n03x5               g012(.a(new_n107), .b(\b[7] ), .c(\a[8] ), .out0(new_n108));
  oaib12aa1n09x5               g013(.a(new_n108), .b(\b[7] ), .c(new_n106), .out0(new_n109));
  oab012aa1n04x5               g014(.a(new_n109), .b(new_n103), .c(new_n105), .out0(new_n110));
  nor042aa1n06x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nand02aa1d24x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  norb02aa1n12x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nor002aa1n20x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nand42aa1d28x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  norb02aa1n15x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nor002aa1n16x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  aoi022aa1d24x5               g022(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n118));
  oai112aa1n06x5               g023(.a(new_n113), .b(new_n116), .c(new_n117), .d(new_n118), .o1(new_n119));
  tech160nm_fiaoi012aa1n05x5   g024(.a(new_n111), .b(new_n114), .c(new_n112), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n119), .b(new_n120), .o1(new_n121));
  xorc02aa1n12x5               g026(.a(\a[6] ), .b(\b[5] ), .out0(new_n122));
  xorc02aa1n12x5               g027(.a(\a[5] ), .b(\b[4] ), .out0(new_n123));
  nand02aa1n04x5               g028(.a(new_n123), .b(new_n122), .o1(new_n124));
  nona22aa1n02x4               g029(.a(new_n121), .b(new_n124), .c(new_n103), .out0(new_n125));
  aoi022aa1n06x5               g030(.a(new_n125), .b(new_n110), .c(\a[9] ), .d(\b[8] ), .o1(new_n126));
  nano22aa1n03x7               g031(.a(new_n126), .b(new_n98), .c(new_n100), .out0(new_n127));
  oai012aa1n02x5               g032(.a(new_n97), .b(new_n126), .c(new_n99), .o1(new_n128));
  nanb02aa1n02x5               g033(.a(new_n127), .b(new_n128), .out0(\s[10] ));
  and002aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  nor042aa1d18x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand02aa1d28x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  nano22aa1n03x7               g039(.a(new_n127), .b(new_n131), .c(new_n134), .out0(new_n135));
  oab012aa1n02x4               g040(.a(new_n134), .b(new_n127), .c(new_n130), .out0(new_n136));
  norp02aa1n02x5               g041(.a(new_n136), .b(new_n135), .o1(\s[11] ));
  nor042aa1n12x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand02aa1d28x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  tech160nm_fioai012aa1n04x5   g045(.a(new_n140), .b(new_n135), .c(new_n132), .o1(new_n141));
  inv020aa1n02x5               g046(.a(new_n135), .o1(new_n142));
  nona22aa1n02x4               g047(.a(new_n142), .b(new_n140), .c(new_n132), .out0(new_n143));
  nanp02aa1n02x5               g048(.a(new_n143), .b(new_n141), .o1(\s[12] ));
  nano23aa1n09x5               g049(.a(new_n132), .b(new_n138), .c(new_n139), .d(new_n133), .out0(new_n145));
  oaoi03aa1n09x5               g050(.a(\a[10] ), .b(\b[9] ), .c(new_n100), .o1(new_n146));
  aoi012aa1d24x5               g051(.a(new_n138), .b(new_n132), .c(new_n139), .o1(new_n147));
  aobi12aa1n02x5               g052(.a(new_n147), .b(new_n145), .c(new_n146), .out0(new_n148));
  xnrc02aa1n03x5               g053(.a(\b[8] ), .b(\a[9] ), .out0(new_n149));
  nona22aa1n09x5               g054(.a(new_n145), .b(new_n149), .c(new_n97), .out0(new_n150));
  aoai13aa1n06x5               g055(.a(new_n148), .b(new_n150), .c(new_n125), .d(new_n110), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1d18x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand02aa1d24x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n151), .c(new_n154), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oabi12aa1n18x5               g061(.a(new_n109), .b(new_n103), .c(new_n105), .out0(new_n157));
  aoi112aa1n06x5               g062(.a(new_n124), .b(new_n103), .c(new_n119), .d(new_n120), .o1(new_n158));
  oabi12aa1n02x5               g063(.a(new_n150), .b(new_n158), .c(new_n157), .out0(new_n159));
  nor042aa1d18x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand02aa1d28x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1d15x5               g066(.a(new_n153), .b(new_n160), .c(new_n161), .d(new_n154), .out0(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  aoi012aa1n06x5               g068(.a(new_n160), .b(new_n153), .c(new_n161), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n159), .d(new_n148), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand02aa1d24x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  nor002aa1n10x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand02aa1d16x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n167), .c(new_n165), .d(new_n169), .o1(new_n173));
  inv020aa1n03x5               g078(.a(new_n164), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n169), .b(new_n174), .c(new_n151), .d(new_n162), .o1(new_n175));
  nona22aa1n02x4               g080(.a(new_n175), .b(new_n172), .c(new_n167), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n173), .b(new_n176), .o1(\s[16] ));
  inv000aa1d42x5               g082(.a(\a[17] ), .o1(new_n178));
  nand02aa1n03x5               g083(.a(new_n145), .b(new_n146), .o1(new_n179));
  nano23aa1n09x5               g084(.a(new_n167), .b(new_n170), .c(new_n171), .d(new_n168), .out0(new_n180));
  tech160nm_fiao0012aa1n02p5x5 g085(.a(new_n170), .b(new_n167), .c(new_n171), .o(new_n181));
  tech160nm_fiaoi012aa1n03p5x5 g086(.a(new_n181), .b(new_n180), .c(new_n174), .o1(new_n182));
  nand02aa1d08x5               g087(.a(new_n180), .b(new_n162), .o1(new_n183));
  aoai13aa1n06x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .d(new_n147), .o1(new_n184));
  nor042aa1n03x5               g089(.a(new_n150), .b(new_n183), .o1(new_n185));
  oaoi13aa1n12x5               g090(.a(new_n184), .b(new_n185), .c(new_n158), .d(new_n157), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(new_n178), .out0(\s[17] ));
  oaoi03aa1n03x5               g092(.a(\a[17] ), .b(\b[16] ), .c(new_n186), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g094(.a(\a[18] ), .o1(new_n190));
  xroi22aa1d06x4               g095(.a(new_n178), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n191));
  aoi112aa1n03x5               g096(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n192));
  aoib12aa1n12x5               g097(.a(new_n192), .b(new_n190), .c(\b[17] ), .out0(new_n193));
  oaib12aa1n06x5               g098(.a(new_n193), .b(new_n186), .c(new_n191), .out0(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nand02aa1d08x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  nor022aa1n08x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand22aa1n09x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  aoai13aa1n03x5               g107(.a(new_n202), .b(new_n197), .c(new_n194), .d(new_n199), .o1(new_n203));
  oai012aa1n02x7               g108(.a(new_n185), .b(new_n158), .c(new_n157), .o1(new_n204));
  nanb02aa1n06x5               g109(.a(new_n184), .b(new_n204), .out0(new_n205));
  nor042aa1n03x5               g110(.a(\b[16] ), .b(\a[17] ), .o1(new_n206));
  aob012aa1n02x5               g111(.a(new_n206), .b(\b[17] ), .c(\a[18] ), .out0(new_n207));
  oaib12aa1n06x5               g112(.a(new_n207), .b(\b[17] ), .c(new_n190), .out0(new_n208));
  aoai13aa1n02x7               g113(.a(new_n199), .b(new_n208), .c(new_n205), .d(new_n191), .o1(new_n209));
  nona22aa1n02x4               g114(.a(new_n209), .b(new_n202), .c(new_n197), .out0(new_n210));
  nanp02aa1n03x5               g115(.a(new_n203), .b(new_n210), .o1(\s[20] ));
  nona23aa1d18x5               g116(.a(new_n201), .b(new_n198), .c(new_n197), .d(new_n200), .out0(new_n212));
  ao0012aa1n03x7               g117(.a(new_n200), .b(new_n197), .c(new_n201), .o(new_n213));
  oabi12aa1n06x5               g118(.a(new_n213), .b(new_n212), .c(new_n193), .out0(new_n214));
  norb02aa1n12x5               g119(.a(new_n191), .b(new_n212), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  oabi12aa1n06x5               g121(.a(new_n214), .b(new_n186), .c(new_n216), .out0(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  xnrc02aa1n12x5               g124(.a(\b[20] ), .b(\a[21] ), .out0(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  tech160nm_fixnrc02aa1n04x5   g126(.a(\b[21] ), .b(\a[22] ), .out0(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n219), .c(new_n217), .d(new_n221), .o1(new_n223));
  aoai13aa1n02x5               g128(.a(new_n221), .b(new_n214), .c(new_n205), .d(new_n215), .o1(new_n224));
  nona22aa1n03x5               g129(.a(new_n224), .b(new_n222), .c(new_n219), .out0(new_n225));
  nanp02aa1n03x5               g130(.a(new_n223), .b(new_n225), .o1(\s[22] ));
  nano23aa1n06x5               g131(.a(new_n197), .b(new_n200), .c(new_n201), .d(new_n198), .out0(new_n227));
  nor042aa1n06x5               g132(.a(new_n222), .b(new_n220), .o1(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n213), .c(new_n227), .d(new_n208), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\a[22] ), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\b[21] ), .o1(new_n231));
  oaoi03aa1n12x5               g136(.a(new_n230), .b(new_n231), .c(new_n219), .o1(new_n232));
  nanp02aa1n02x5               g137(.a(new_n229), .b(new_n232), .o1(new_n233));
  nand03aa1n02x5               g138(.a(new_n191), .b(new_n228), .c(new_n227), .o1(new_n234));
  oabi12aa1n06x5               g139(.a(new_n233), .b(new_n186), .c(new_n234), .out0(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  xorc02aa1n12x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  tech160nm_fixnrc02aa1n05x5   g143(.a(\b[23] ), .b(\a[24] ), .out0(new_n239));
  aoai13aa1n02x7               g144(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n240));
  nand42aa1n03x5               g145(.a(new_n235), .b(new_n238), .o1(new_n241));
  nona22aa1n03x5               g146(.a(new_n241), .b(new_n239), .c(new_n237), .out0(new_n242));
  nanp02aa1n03x5               g147(.a(new_n242), .b(new_n240), .o1(\s[24] ));
  norb02aa1n02x7               g148(.a(new_n238), .b(new_n239), .out0(new_n244));
  nanb02aa1n03x5               g149(.a(new_n234), .b(new_n244), .out0(new_n245));
  inv000aa1n02x5               g150(.a(new_n244), .o1(new_n246));
  orn002aa1n02x5               g151(.a(\a[23] ), .b(\b[22] ), .o(new_n247));
  oao003aa1n02x5               g152(.a(\a[24] ), .b(\b[23] ), .c(new_n247), .carry(new_n248));
  aoai13aa1n03x5               g153(.a(new_n248), .b(new_n246), .c(new_n229), .d(new_n232), .o1(new_n249));
  oabi12aa1n06x5               g154(.a(new_n249), .b(new_n186), .c(new_n245), .out0(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n12x5               g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  xnrc02aa1n12x5               g158(.a(\b[25] ), .b(\a[26] ), .out0(new_n254));
  aoai13aa1n02x7               g159(.a(new_n254), .b(new_n252), .c(new_n250), .d(new_n253), .o1(new_n255));
  nand42aa1n03x5               g160(.a(new_n250), .b(new_n253), .o1(new_n256));
  nona22aa1n03x5               g161(.a(new_n256), .b(new_n254), .c(new_n252), .out0(new_n257));
  nanp02aa1n03x5               g162(.a(new_n257), .b(new_n255), .o1(\s[26] ));
  norb02aa1n06x5               g163(.a(new_n253), .b(new_n254), .out0(new_n259));
  inv000aa1d42x5               g164(.a(\a[26] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(\b[25] ), .o1(new_n261));
  oaoi03aa1n12x5               g166(.a(new_n260), .b(new_n261), .c(new_n252), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  aoi012aa1n02x5               g168(.a(new_n263), .b(new_n249), .c(new_n259), .o1(new_n264));
  nano22aa1n03x7               g169(.a(new_n234), .b(new_n244), .c(new_n259), .out0(new_n265));
  oaib12aa1n06x5               g170(.a(new_n264), .b(new_n186), .c(new_n265), .out0(new_n266));
  xorb03aa1n03x5               g171(.a(new_n266), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  xorc02aa1n02x5               g173(.a(\a[27] ), .b(\b[26] ), .out0(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[27] ), .b(\a[28] ), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n270), .b(new_n268), .c(new_n266), .d(new_n269), .o1(new_n271));
  inv000aa1n02x5               g176(.a(new_n232), .o1(new_n272));
  aoai13aa1n03x5               g177(.a(new_n244), .b(new_n272), .c(new_n214), .d(new_n228), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n259), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n262), .b(new_n274), .c(new_n273), .d(new_n248), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n269), .b(new_n275), .c(new_n205), .d(new_n265), .o1(new_n276));
  nona22aa1n02x4               g181(.a(new_n276), .b(new_n270), .c(new_n268), .out0(new_n277));
  nanp02aa1n03x5               g182(.a(new_n271), .b(new_n277), .o1(\s[28] ));
  norb02aa1n02x5               g183(.a(new_n269), .b(new_n270), .out0(new_n279));
  aoai13aa1n02x5               g184(.a(new_n279), .b(new_n275), .c(new_n205), .d(new_n265), .o1(new_n280));
  aob012aa1n03x5               g185(.a(new_n268), .b(\b[27] ), .c(\a[28] ), .out0(new_n281));
  oa0012aa1n12x5               g186(.a(new_n281), .b(\b[27] ), .c(\a[28] ), .o(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  nona22aa1n02x4               g189(.a(new_n280), .b(new_n283), .c(new_n284), .out0(new_n285));
  aoai13aa1n03x5               g190(.a(new_n284), .b(new_n283), .c(new_n266), .d(new_n279), .o1(new_n286));
  nanp02aa1n03x5               g191(.a(new_n286), .b(new_n285), .o1(\s[29] ));
  nanp02aa1n02x5               g192(.a(\b[0] ), .b(\a[1] ), .o1(new_n288));
  xorb03aa1n02x5               g193(.a(new_n288), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n269), .b(new_n284), .c(new_n270), .out0(new_n290));
  oaoi03aa1n09x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .o1(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[29] ), .b(\a[30] ), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n291), .c(new_n266), .d(new_n290), .o1(new_n293));
  aoai13aa1n02x5               g198(.a(new_n290), .b(new_n275), .c(new_n205), .d(new_n265), .o1(new_n294));
  nona22aa1n02x4               g199(.a(new_n294), .b(new_n291), .c(new_n292), .out0(new_n295));
  nanp02aa1n03x5               g200(.a(new_n293), .b(new_n295), .o1(\s[30] ));
  nanb02aa1n02x5               g201(.a(new_n292), .b(new_n291), .out0(new_n297));
  oai012aa1n02x7               g202(.a(new_n297), .b(\b[29] ), .c(\a[30] ), .o1(new_n298));
  norb02aa1n02x5               g203(.a(new_n290), .b(new_n292), .out0(new_n299));
  aoai13aa1n04x5               g204(.a(new_n299), .b(new_n275), .c(new_n205), .d(new_n265), .o1(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  nona22aa1n02x4               g206(.a(new_n300), .b(new_n301), .c(new_n298), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n301), .b(new_n298), .c(new_n266), .d(new_n299), .o1(new_n303));
  nanp02aa1n03x5               g208(.a(new_n303), .b(new_n302), .o1(\s[31] ));
  norp02aa1n02x5               g209(.a(new_n118), .b(new_n117), .o1(new_n305));
  xnrc02aa1n02x5               g210(.a(new_n305), .b(new_n116), .out0(\s[3] ));
  oaoi13aa1n02x5               g211(.a(new_n114), .b(new_n115), .c(new_n118), .d(new_n117), .o1(new_n307));
  xnrc02aa1n02x5               g212(.a(new_n307), .b(new_n113), .out0(\s[4] ));
  xnbna2aa1n03x5               g213(.a(new_n123), .b(new_n119), .c(new_n120), .out0(\s[5] ));
  orn002aa1n02x5               g214(.a(\a[5] ), .b(\b[4] ), .o(new_n310));
  nanp02aa1n02x5               g215(.a(new_n121), .b(new_n123), .o1(new_n311));
  xnbna2aa1n03x5               g216(.a(new_n122), .b(new_n311), .c(new_n310), .out0(\s[6] ));
  aoai13aa1n02x5               g217(.a(new_n105), .b(new_n124), .c(new_n119), .d(new_n120), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g219(.a(new_n107), .b(new_n313), .c(new_n102), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[7] ), .c(new_n106), .out0(\s[8] ));
  xobna2aa1n03x5               g221(.a(new_n149), .b(new_n125), .c(new_n110), .out0(\s[9] ));
endmodule


