// Benchmark "adder" written by ABC on Thu Jul 18 03:02:07 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n135, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n310, new_n312, new_n313,
    new_n315;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nand42aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor022aa1n16x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nor002aa1n12x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  oai012aa1n03x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  nor042aa1n03x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand22aa1n04x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  nand22aa1n06x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  tech160nm_fiaoi012aa1n05x5   g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  inv000aa1n03x5               g011(.a(new_n106), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nano23aa1n06x5               g013(.a(new_n101), .b(new_n100), .c(new_n108), .d(new_n99), .out0(new_n109));
  aobi12aa1n06x5               g014(.a(new_n102), .b(new_n109), .c(new_n107), .out0(new_n110));
  xnrc02aa1n03x5               g015(.a(\b[5] ), .b(\a[6] ), .out0(new_n111));
  tech160nm_fixnrc02aa1n02p5x5 g016(.a(\b[4] ), .b(\a[5] ), .out0(new_n112));
  nanp02aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nor022aa1n16x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  tech160nm_finand02aa1n05x5   g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nor022aa1n08x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nano23aa1n02x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  nona22aa1n02x4               g022(.a(new_n117), .b(new_n112), .c(new_n111), .out0(new_n118));
  orn002aa1n12x5               g023(.a(\a[5] ), .b(\b[4] ), .o(new_n119));
  oaoi03aa1n09x5               g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n115), .b(new_n116), .c(new_n114), .o1(new_n121));
  aobi12aa1n02x7               g026(.a(new_n121), .b(new_n117), .c(new_n120), .out0(new_n122));
  oai012aa1n04x7               g027(.a(new_n122), .b(new_n110), .c(new_n118), .o1(new_n123));
  xnrc02aa1n03x5               g028(.a(\b[8] ), .b(\a[9] ), .out0(new_n124));
  aoib12aa1n02x5               g029(.a(new_n98), .b(new_n123), .c(new_n124), .out0(new_n125));
  xorb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  inv000aa1d42x5               g031(.a(\b[9] ), .o1(new_n127));
  oao003aa1n02x5               g032(.a(new_n97), .b(new_n127), .c(new_n98), .carry(new_n128));
  xnrc02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .out0(new_n129));
  nor042aa1n03x5               g034(.a(new_n129), .b(new_n124), .o1(new_n130));
  aoi012aa1n02x5               g035(.a(new_n128), .b(new_n123), .c(new_n130), .o1(new_n131));
  xnrb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g037(.a(\a[11] ), .b(\b[10] ), .c(new_n131), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nona23aa1n02x4               g039(.a(new_n99), .b(new_n108), .c(new_n101), .d(new_n100), .out0(new_n135));
  tech160nm_fioai012aa1n05x5   g040(.a(new_n102), .b(new_n135), .c(new_n106), .o1(new_n136));
  nona23aa1n03x5               g041(.a(new_n115), .b(new_n113), .c(new_n116), .d(new_n114), .out0(new_n137));
  nor043aa1n02x5               g042(.a(new_n137), .b(new_n112), .c(new_n111), .o1(new_n138));
  oaib12aa1n06x5               g043(.a(new_n121), .b(new_n137), .c(new_n120), .out0(new_n139));
  nor042aa1n06x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nanp02aa1n04x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nor022aa1n08x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nona23aa1n03x5               g048(.a(new_n143), .b(new_n141), .c(new_n140), .d(new_n142), .out0(new_n144));
  nor003aa1n02x5               g049(.a(new_n144), .b(new_n129), .c(new_n124), .o1(new_n145));
  aoai13aa1n03x5               g050(.a(new_n145), .b(new_n139), .c(new_n136), .d(new_n138), .o1(new_n146));
  nano23aa1n03x7               g051(.a(new_n140), .b(new_n142), .c(new_n143), .d(new_n141), .out0(new_n147));
  aoi012aa1n02x7               g052(.a(new_n142), .b(new_n140), .c(new_n143), .o1(new_n148));
  aobi12aa1n06x5               g053(.a(new_n148), .b(new_n147), .c(new_n128), .out0(new_n149));
  nor042aa1n06x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nand42aa1d28x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  norb02aa1n02x5               g056(.a(new_n151), .b(new_n150), .out0(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n152), .b(new_n146), .c(new_n149), .out0(\s[13] ));
  oaoi03aa1n02x5               g058(.a(new_n97), .b(new_n127), .c(new_n98), .o1(new_n154));
  tech160nm_fioai012aa1n05x5   g059(.a(new_n148), .b(new_n144), .c(new_n154), .o1(new_n155));
  aoai13aa1n02x5               g060(.a(new_n152), .b(new_n155), .c(new_n123), .d(new_n145), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n150), .out0(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n06x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n24x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nano23aa1d15x5               g065(.a(new_n150), .b(new_n159), .c(new_n160), .d(new_n151), .out0(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  tech160nm_fioai012aa1n03p5x5 g067(.a(new_n160), .b(new_n159), .c(new_n150), .o1(new_n163));
  aoai13aa1n04x5               g068(.a(new_n163), .b(new_n162), .c(new_n146), .d(new_n149), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n09x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  xorc02aa1n12x5               g071(.a(\a[15] ), .b(\b[14] ), .out0(new_n167));
  xorc02aa1n12x5               g072(.a(\a[16] ), .b(\b[15] ), .out0(new_n168));
  aoi112aa1n02x5               g073(.a(new_n166), .b(new_n168), .c(new_n164), .d(new_n167), .o1(new_n169));
  aoai13aa1n03x5               g074(.a(new_n168), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n170));
  norb02aa1n02x7               g075(.a(new_n170), .b(new_n169), .out0(\s[16] ));
  nand23aa1d12x5               g076(.a(new_n161), .b(new_n167), .c(new_n168), .o1(new_n172));
  nano22aa1d15x5               g077(.a(new_n172), .b(new_n130), .c(new_n147), .out0(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n139), .c(new_n136), .d(new_n138), .o1(new_n174));
  xnrc02aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .out0(new_n175));
  xnrc02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .out0(new_n176));
  inv000aa1n02x5               g081(.a(new_n166), .o1(new_n177));
  oao003aa1n03x5               g082(.a(\a[16] ), .b(\b[15] ), .c(new_n177), .carry(new_n178));
  oai013aa1n03x5               g083(.a(new_n178), .b(new_n175), .c(new_n176), .d(new_n163), .o1(new_n179));
  aoib12aa1n12x5               g084(.a(new_n179), .b(new_n155), .c(new_n172), .out0(new_n180));
  nanp02aa1n09x5               g085(.a(new_n174), .b(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g087(.a(\a[18] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n03x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  xroi22aa1d06x4               g092(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n188));
  oai022aa1d24x5               g093(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n189));
  oaib12aa1n18x5               g094(.a(new_n189), .b(new_n183), .c(\b[17] ), .out0(new_n190));
  inv000aa1n09x5               g095(.a(new_n190), .o1(new_n191));
  nor002aa1d32x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nanp02aa1n09x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n188), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n188), .o1(new_n196));
  norb02aa1n02x7               g101(.a(new_n195), .b(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n24x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nand42aa1n16x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  nona22aa1n02x5               g106(.a(new_n195), .b(new_n201), .c(new_n192), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n192), .o1(new_n203));
  aobi12aa1n06x5               g108(.a(new_n201), .b(new_n195), .c(new_n203), .out0(new_n204));
  norb02aa1n03x4               g109(.a(new_n202), .b(new_n204), .out0(\s[20] ));
  nano23aa1n09x5               g110(.a(new_n192), .b(new_n199), .c(new_n200), .d(new_n193), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n188), .b(new_n206), .o1(new_n207));
  nona23aa1n09x5               g112(.a(new_n200), .b(new_n193), .c(new_n192), .d(new_n199), .out0(new_n208));
  aoi012aa1n12x5               g113(.a(new_n199), .b(new_n192), .c(new_n200), .o1(new_n209));
  oai012aa1n18x5               g114(.a(new_n209), .b(new_n208), .c(new_n190), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoai13aa1n04x5               g116(.a(new_n211), .b(new_n207), .c(new_n174), .d(new_n180), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n03x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  xorc02aa1n02x5               g119(.a(\a[21] ), .b(\b[20] ), .out0(new_n215));
  xorc02aa1n02x5               g120(.a(\a[22] ), .b(\b[21] ), .out0(new_n216));
  aoi112aa1n02x5               g121(.a(new_n214), .b(new_n216), .c(new_n212), .d(new_n215), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n216), .b(new_n214), .c(new_n212), .d(new_n215), .o1(new_n218));
  norb02aa1n02x7               g123(.a(new_n218), .b(new_n217), .out0(\s[22] ));
  inv000aa1d42x5               g124(.a(\a[21] ), .o1(new_n220));
  inv040aa1d32x5               g125(.a(\a[22] ), .o1(new_n221));
  xroi22aa1d06x4               g126(.a(new_n220), .b(\b[20] ), .c(new_n221), .d(\b[21] ), .out0(new_n222));
  nanp03aa1n02x5               g127(.a(new_n222), .b(new_n188), .c(new_n206), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\b[21] ), .o1(new_n224));
  oaoi03aa1n12x5               g129(.a(new_n221), .b(new_n224), .c(new_n214), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoi012aa1n02x5               g131(.a(new_n226), .b(new_n210), .c(new_n222), .o1(new_n227));
  aoai13aa1n04x5               g132(.a(new_n227), .b(new_n223), .c(new_n174), .d(new_n180), .o1(new_n228));
  xorb03aa1n02x5               g133(.a(new_n228), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g134(.a(\b[22] ), .b(\a[23] ), .o1(new_n230));
  xorc02aa1n12x5               g135(.a(\a[23] ), .b(\b[22] ), .out0(new_n231));
  xorc02aa1n12x5               g136(.a(\a[24] ), .b(\b[23] ), .out0(new_n232));
  aoi112aa1n02x5               g137(.a(new_n230), .b(new_n232), .c(new_n228), .d(new_n231), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n232), .b(new_n230), .c(new_n228), .d(new_n231), .o1(new_n234));
  norb02aa1n02x7               g139(.a(new_n234), .b(new_n233), .out0(\s[24] ));
  oabi12aa1n06x5               g140(.a(new_n179), .b(new_n149), .c(new_n172), .out0(new_n236));
  and002aa1n02x5               g141(.a(new_n232), .b(new_n231), .o(new_n237));
  inv030aa1n02x5               g142(.a(new_n237), .o1(new_n238));
  nano32aa1n02x4               g143(.a(new_n238), .b(new_n222), .c(new_n188), .d(new_n206), .out0(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n236), .c(new_n123), .d(new_n173), .o1(new_n240));
  inv020aa1n03x5               g145(.a(new_n209), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n222), .b(new_n241), .c(new_n206), .d(new_n191), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n243));
  oab012aa1n04x5               g148(.a(new_n243), .b(\a[24] ), .c(\b[23] ), .out0(new_n244));
  aoai13aa1n12x5               g149(.a(new_n244), .b(new_n238), .c(new_n242), .d(new_n225), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  xnrc02aa1n12x5               g151(.a(\b[24] ), .b(\a[25] ), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  xnbna2aa1n03x5               g153(.a(new_n248), .b(new_n240), .c(new_n246), .out0(\s[25] ));
  nor042aa1n03x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n248), .b(new_n245), .c(new_n181), .d(new_n239), .o1(new_n252));
  xnrc02aa1n06x5               g157(.a(\b[25] ), .b(\a[26] ), .out0(new_n253));
  nanp03aa1n03x5               g158(.a(new_n252), .b(new_n251), .c(new_n253), .o1(new_n254));
  tech160nm_fiaoi012aa1n02p5x5 g159(.a(new_n253), .b(new_n252), .c(new_n251), .o1(new_n255));
  norb02aa1n03x4               g160(.a(new_n254), .b(new_n255), .out0(\s[26] ));
  nor042aa1n06x5               g161(.a(new_n253), .b(new_n247), .o1(new_n257));
  nano22aa1n03x7               g162(.a(new_n223), .b(new_n237), .c(new_n257), .out0(new_n258));
  aoai13aa1n04x5               g163(.a(new_n258), .b(new_n236), .c(new_n123), .d(new_n173), .o1(new_n259));
  nand02aa1d10x5               g164(.a(new_n245), .b(new_n257), .o1(new_n260));
  oao003aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .c(new_n251), .carry(new_n261));
  xorc02aa1n12x5               g166(.a(\a[27] ), .b(\b[26] ), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  aoi013aa1n06x4               g168(.a(new_n263), .b(new_n259), .c(new_n260), .d(new_n261), .o1(new_n264));
  aobi12aa1n06x5               g169(.a(new_n258), .b(new_n174), .c(new_n180), .out0(new_n265));
  aoai13aa1n02x7               g170(.a(new_n237), .b(new_n226), .c(new_n210), .d(new_n222), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n257), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n261), .b(new_n267), .c(new_n266), .d(new_n244), .o1(new_n268));
  norp03aa1n02x5               g173(.a(new_n268), .b(new_n265), .c(new_n262), .o1(new_n269));
  nor002aa1n02x5               g174(.a(new_n264), .b(new_n269), .o1(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  inv040aa1n03x5               g176(.a(new_n271), .o1(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  nano22aa1n03x5               g178(.a(new_n264), .b(new_n272), .c(new_n273), .out0(new_n274));
  oaih12aa1n02x5               g179(.a(new_n262), .b(new_n268), .c(new_n265), .o1(new_n275));
  aoi012aa1n02x7               g180(.a(new_n273), .b(new_n275), .c(new_n272), .o1(new_n276));
  norp02aa1n03x5               g181(.a(new_n276), .b(new_n274), .o1(\s[28] ));
  xnrc02aa1n02x5               g182(.a(\b[28] ), .b(\a[29] ), .out0(new_n278));
  norb02aa1n02x5               g183(.a(new_n262), .b(new_n273), .out0(new_n279));
  inv000aa1n02x5               g184(.a(new_n279), .o1(new_n280));
  aoi013aa1n02x5               g185(.a(new_n280), .b(new_n259), .c(new_n260), .d(new_n261), .o1(new_n281));
  oao003aa1n02x5               g186(.a(\a[28] ), .b(\b[27] ), .c(new_n272), .carry(new_n282));
  nano22aa1n03x5               g187(.a(new_n281), .b(new_n278), .c(new_n282), .out0(new_n283));
  oaih12aa1n02x5               g188(.a(new_n279), .b(new_n268), .c(new_n265), .o1(new_n284));
  tech160nm_fiaoi012aa1n02p5x5 g189(.a(new_n278), .b(new_n284), .c(new_n282), .o1(new_n285));
  norp02aa1n03x5               g190(.a(new_n285), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g191(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n06x5               g192(.a(new_n262), .b(new_n278), .c(new_n273), .out0(new_n288));
  oaih12aa1n02x5               g193(.a(new_n288), .b(new_n268), .c(new_n265), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[29] ), .b(\a[30] ), .out0(new_n291));
  tech160nm_fiaoi012aa1n03p5x5 g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n288), .o1(new_n293));
  aoi013aa1n06x4               g198(.a(new_n293), .b(new_n259), .c(new_n260), .d(new_n261), .o1(new_n294));
  nano22aa1n02x5               g199(.a(new_n294), .b(new_n290), .c(new_n291), .out0(new_n295));
  nor002aa1n02x5               g200(.a(new_n292), .b(new_n295), .o1(\s[30] ));
  xnrc02aa1n02x5               g201(.a(\b[30] ), .b(\a[31] ), .out0(new_n297));
  norb02aa1n03x4               g202(.a(new_n288), .b(new_n291), .out0(new_n298));
  inv020aa1n02x5               g203(.a(new_n298), .o1(new_n299));
  aoi013aa1n02x5               g204(.a(new_n299), .b(new_n259), .c(new_n260), .d(new_n261), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .c(new_n290), .carry(new_n301));
  nano22aa1n03x5               g206(.a(new_n300), .b(new_n297), .c(new_n301), .out0(new_n302));
  oaih12aa1n02x5               g207(.a(new_n298), .b(new_n268), .c(new_n265), .o1(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n297), .b(new_n303), .c(new_n301), .o1(new_n304));
  norp02aa1n03x5               g209(.a(new_n304), .b(new_n302), .o1(\s[31] ));
  xnrb03aa1n02x5               g210(.a(new_n106), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g211(.a(\a[3] ), .b(\b[2] ), .c(new_n106), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g213(.a(new_n136), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g214(.a(\a[5] ), .b(\b[4] ), .c(new_n110), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  orn002aa1n02x5               g216(.a(new_n111), .b(new_n112), .o(new_n312));
  oabi12aa1n02x5               g217(.a(new_n120), .b(new_n110), .c(new_n312), .out0(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g219(.a(new_n116), .b(new_n313), .c(new_n113), .o1(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g221(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


